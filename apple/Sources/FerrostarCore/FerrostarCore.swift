import CoreLocation
import FerrostarCoreFFI
import Foundation

enum FerrostarCoreError: Error, Equatable {
    /// The user has disabled location services for this app.
    case locationServicesDisabled
    case userLocationUnknown
    /// The route request from the route adapter has an invalid URL.
    ///
    /// This should never be encountered by end users of the library, and indicates a programming error
    /// in the route adapter.
    case invalidRequestUrl
    /// Invalid (non-2xx) HTTP status
    case httpStatusCode(Int)
    /// A resumable cached session was not found. Enable navigation session caching.
    case noCachedSession
}

/// Corrective action to take when the user deviates from the route.
public enum CorrectiveAction {
    /// Don't do anything.
    ///
    /// Note that this is most commonly paired with no route deviation tracking as a formality.
    /// Think twice before using this as a mechanism for implementing your own logic outside of the provided framework,
    /// as doing so will mean you miss out on state updates around alternate route calculation.
    case doNothing
    /// Tells the core to fetch new routes from the route adapter.
    ///
    /// Once they are available, the delegate will be notified of the new routes.
    case getNewRoutes(waypoints: [Waypoint])
}

/// Receives events from ``FerrostarCore``.
///
/// This is the central point responsible for relaying updates back to the application.
public protocol FerrostarCoreDelegate: AnyObject {
    /// Called when navigation is started on a specific route.
    func core(_ core: FerrostarCore, didStartWith route: Route)

    /// Called when the core detects that the user has deviated from the route.
    ///
    /// This hook enables app developers to take the most appropriate corrective action.
    func core(
        _ core: FerrostarCore,
        correctiveActionForDeviation deviationInMeters: Double,
        remainingWaypoints waypoints: [Waypoint]
    ) -> CorrectiveAction

    /// Called when the core has loaded alternate routes.
    ///
    /// The developer may decide whether or not to act on this information given the current trip state.
    /// This is currently used for recalculation when the user diverges from the route, but can be extended for other
    /// uses in the future.
    /// Note that the `isCalculatingNewRoute` property of ``NavigationState`` will be true until this method returns.
    /// Delegates may thus rely on this state introspection to decide what action to take given alternate routes.
    func core(_ core: FerrostarCore, loadedAlternateRoutes routes: [Route])
}

/// This is the entrypoint for end users of Ferrostar on iOS, and is responsible
/// for "driving" the navigation with location updates and other events.
///
/// The usual flow is for callers to configure an instance of the core, set a ``delegate``,
/// and reuse the instance for as long as it makes sense (necessarily somewhat app-specific).
/// You can first call ``getRoutes(initialLocation:waypoints:)``
/// to fetch a list of possible routes asynchronously. After selecting a suitable route (either interactively by the
/// user, or programmatically), call ``startNavigation(route:config:)`` to start a session.
///
/// NOTE: it is the responsibility of the caller to ensure that the location provider is authorized to get
/// live user location with high precision.
///
/// Threading (Stegra fork): each location fix's ``NavigationSession`` update runs on a private serial queue, not on
/// the thread that delivers the fix. The update moves the whole `NavState` (every remaining step and coordinate)
/// across the FFI twice, so it costs O(remaining route); on a long route it blocked the main thread for 150-200 ms
/// per fix in Debug builds. Updates run one at a time, in order, each from the previous update's state; a fix that
/// arrives while an update runs waits, and a newer fix replaces it (coalescing). New state is still published on
/// the main thread, in order: a state older than one already published is dropped (before, the start state could
/// be re-published after the first fix's), and so are results from a session that was stopped or replaced meanwhile.
/// Navigation observers and custom route deviation detectors are therefore called on that queue.
// TODO: See about making FerrostarCore its own actor; then we can verify that we've published things back on the main actor. Need to see if this is possible with obj-c interop. See https://github.com/apple/swift-evolution/blob/main/proposals/0306-actors.md#actor-interoperability-with-objective-c
@objc public class FerrostarCore: NSObject {
    /// The delegate which will receive Ferrostar core events.
    public weak var delegate: FerrostarCoreDelegate?

    /// The spoken instruction observer; responsible for text-to-speech announcements.
    public let spokenInstructionObserver: SpokenInstructionObserver

    /// The minimum time to wait before initiating another route recalculation.
    ///
    /// This matters in the case that a user is off route, the framework calculates a new route,
    /// and the user is determined to still be off the new route.
    /// This adds a minimum delay (default 5 seconds).
    public var minimumTimeBeforeRecalculation: TimeInterval = 5

    /// The minimum distance (in meters) the user must move before performing another route recalculation.
    ///
    /// This ensures that, while the user remains off the route, we don't keep triggering useless recalculations.
    public var minimumMovementBeforeRecalculation = CLLocationDistance(50)

    /// The observable state of the model (for easy binding in SwiftUI views).
    @Published private var coreNavState: NavState?
    @Published public private(set) var state: NavigationState?
    @Published public private(set) var route: Route?

    public let annotation: (any AnnotationPublishing)?
    public let widgetProvider: WidgetProviding?

    private let networkSession: URLRequestLoading
    private let routeProvider: RouteProvider
    private let locationProvider: LocationProviding

    private let sessionBuilder: FerrostarSessionBuilder

    /// The session being navigated and the newest state it produced: the base for the next update.
    private struct UpdateBase {
        let session: NavigationSession
        var state: NavState
        let generation: UInt64
        /// 0 for the start state, +1 for each update.
        var sequence: UInt64 = 0
    }

    /// Runs the per-fix session updates (see the threading note above). Serial: one update at a time, in order.
    private let updateQueue = DispatchQueue(label: "com.stadiamaps.ferrostar.navigation-update", qos: .userInitiated)
    /// Guards `updateBase`, `sessionGeneration`, `pendingLocation` and `drainScheduled`.
    private let updateLock = NSLock()
    private var updateBase: UpdateBase?
    /// Bumped whenever navigation starts, resumes or stops. A result from an older generation is dropped.
    private var sessionGeneration: UInt64 = 0
    /// The newest fix not yet handed to the session. A newer fix replaces it (coalescing).
    private var pendingLocation: UserLocation?
    /// Whether a drain of `pendingLocation` is queued and has not started yet.
    private var drainScheduled = false
    /// Main thread only: the generation and sequence of the last state published by `update`.
    private var publishedGeneration: UInt64 = 0
    private var publishedSequence: UInt64 = 0

    private var routeRequestInFlight = false
    private var lastAutomaticRecalculation: Date?
    private var lastLocation: UserLocation?
    // The last location from which we triggered a recalculation
    private var lastRecalculationLocation: UserLocation?
    private var recalculationTask: Task<Void, Never>?
    private var queuedUtteranceIDs: Set<UUID> = Set()

    public init(
        routeProvider: RouteProvider,
        locationProvider: LocationProviding,
        sessionBuilder: FerrostarSessionBuilder,
        networkSession: URLRequestLoading,
        annotation: (any AnnotationPublishing)? = nil,
        spokenInstructionObserver: SpokenInstructionObserver =
            .initAVSpeechSynthesizer(), // Set up the a standard Apple AV Speech Synth.
        widgetProvider: WidgetProviding? = nil
    ) {
        self.routeProvider = routeProvider
        self.locationProvider = locationProvider
        self.networkSession = networkSession
        self.annotation = annotation
        self.spokenInstructionObserver = spokenInstructionObserver
        self.widgetProvider = widgetProvider

        self.sessionBuilder = sessionBuilder

        super.init()

        // Location provider setup
        locationProvider.delegate = self

        // Annotation publisher setup
        self.annotation?.configure($state)
    }

    /// Initializes a core instance with the given parameters.
    ///
    /// This designated initializer is the most flexible, but the convenience ones may be easier to use.
    /// for common configurations.
    ///
    /// - Parameters:
    ///   - routeProvider: The route provider is responsible for fetching routes from a server or locally.
    ///   - locationProvider: The location provider is responsible for tracking the user's location for navigation trip
    /// updates.
    ///   - navigationControllerConfig: Configure the behavior of the navigation controller.
    ///   - networkSession: The network session to run route fetches on. A custom ``RouteProvider`` may not use this.
    ///   - annotation: An implementation of the annotation publisher that transforms custom annotation JSON into
    /// published values of defined swift types.
    public convenience init(
        routeProvider: RouteProvider,
        locationProvider: LocationProviding,
        navigationControllerConfig: SwiftNavigationControllerConfig,
        networkSession: URLRequestLoading,
        annotation: (any AnnotationPublishing)? = nil,
        spokenInstructionObserver: SpokenInstructionObserver =
            .initAVSpeechSynthesizer(), // Set up the a standard Apple AV Speech Synth.
        widgetProvider: WidgetProviding? = nil
    ) {
        let sessionBuilder = FerrostarSessionBuilder(
            config: navigationControllerConfig
        )

        self.init(
            routeProvider: routeProvider,
            locationProvider: locationProvider,
            sessionBuilder: sessionBuilder,
            networkSession: networkSession,
            annotation: annotation,
            spokenInstructionObserver: spokenInstructionObserver,
            widgetProvider: widgetProvider
        )
    }

    /// Initializes a core instance for a well-known API accessed over HTTP.
    /// This convenience initializer provides easy access for any built-in route provider.
    ///
    /// - Parameters
    ///   - routingEngine: The configuration for a well-known routing engine.
    ///   - navigationControllerConfig: Configuration of the navigation session.
    ///   - options: A dictionary of options to include in the request. The Valhalla request generator sets several
    /// automatically (like `format`), but this lets you add arbitrary options so you can access the full API.
    ///   - networkSession: The network session to use. Don't set this unless you need to replace the networking stack
    /// (ex: for testing).
    ///   - annotation: An implementation of the annotation publisher that transforms custom annotation JSON into
    /// published values of defined swift types.
    public convenience init(
        wellKnownRouteProvider: WellKnownRouteProvider,
        locationProvider: LocationProviding,
        navigationControllerConfig: SwiftNavigationControllerConfig,
        networkSession: URLRequestLoading = URLSession.shared,
        annotation: (any AnnotationPublishing)? = nil,
        spokenInstructionObserver: SpokenInstructionObserver =
            .initAVSpeechSynthesizer(),
        widgetProvider: WidgetProviding? = nil
    ) throws {
        let adapter = try RouteAdapter.fromWellKnownRouteProvider(wellKnownRouteProvider: wellKnownRouteProvider)
        self.init(
            routeProvider: .routeAdapter(adapter),
            locationProvider: locationProvider,
            navigationControllerConfig: navigationControllerConfig,
            networkSession: networkSession,
            annotation: annotation,
            spokenInstructionObserver: spokenInstructionObserver,
            widgetProvider: widgetProvider
        )
    }

    public convenience init(
        routeAdapter: RouteAdapterProtocol,
        locationProvider: LocationProviding,
        navigationControllerConfig: SwiftNavigationControllerConfig,
        networkSession: URLRequestLoading = URLSession.shared,
        annotation: (any AnnotationPublishing)? = nil,
        spokenInstructionObserver: SpokenInstructionObserver =
            .initAVSpeechSynthesizer(),
        widgetProvider: WidgetProviding? = nil
    ) {
        self.init(
            routeProvider: .routeAdapter(routeAdapter),
            locationProvider: locationProvider,
            navigationControllerConfig: navigationControllerConfig,
            networkSession: networkSession,
            annotation: annotation,
            spokenInstructionObserver: spokenInstructionObserver,
            widgetProvider: widgetProvider
        )
    }

    public convenience init(
        customRouteProvider: CustomRouteProvider,
        locationProvider: LocationProviding,
        sessionBuilder: FerrostarSessionBuilder,
        networkSession: URLRequestLoading = URLSession.shared,
        annotation: (any AnnotationPublishing)? = nil,
        spokenInstructionObserver: SpokenInstructionObserver =
            .initAVSpeechSynthesizer(),
        widgetProvider: WidgetProviding? = nil
    ) {
        self.init(
            routeProvider: .customProvider(customRouteProvider),
            locationProvider: locationProvider,
            sessionBuilder: sessionBuilder,
            networkSession: networkSession,
            annotation: annotation,
            spokenInstructionObserver: spokenInstructionObserver,
            widgetProvider: widgetProvider
        )
    }

    public convenience init(
        customRouteProvider: CustomRouteProvider,
        locationProvider: LocationProviding,
        navigationControllerConfig: SwiftNavigationControllerConfig,
        networkSession: URLRequestLoading = URLSession.shared,
        annotation: (any AnnotationPublishing)? = nil,
        spokenInstructionObserver: SpokenInstructionObserver =
            .initAVSpeechSynthesizer(),
        widgetProvider: WidgetProviding? = nil
    ) {
        self.init(
            routeProvider: .customProvider(customRouteProvider),
            locationProvider: locationProvider,
            navigationControllerConfig: navigationControllerConfig,
            networkSession: networkSession,
            annotation: annotation,
            spokenInstructionObserver: spokenInstructionObserver,
            widgetProvider: widgetProvider
        )
    }

    /// Tries to get routes visiting one or more waypoints starting from the initial location.
    ///
    /// Success and failure are communicated via ``delegate`` methods.
    public func getRoutes(initialLocation: UserLocation, waypoints: [Waypoint]) async throws
        -> [Route]
    {
        routeRequestInFlight = true

        defer {
            routeRequestInFlight = false
        }

        switch routeProvider {
        case let .customProvider(provider):
            return try await provider.getRoutes(userLocation: initialLocation, waypoints: waypoints)
        case let .routeAdapter(routeAdapter):
            let routeRequest = try routeAdapter.generateRequest(
                userLocation: initialLocation,
                waypoints: waypoints
            )

            let urlRequest = try routeRequest.urlRequest
            let (data, response) = try await networkSession.loadData(with: urlRequest)

            if let res = response as? HTTPURLResponse, res.statusCode < 200 || res.statusCode >= 300 {
                throw FerrostarCoreError.httpStatusCode(res.statusCode)
            } else {
                return try routeAdapter.parseResponse(response: data)
            }
        }
    }

    /// Starts navigation with the given route. Any previous navigation session is dropped.
    ///
    /// - Parameters:
    ///   - route: The route to navigate.
    ///   - userLocation: The user's location. This should be as close to the users location and the start of the route
    /// as possible. If the location is too stale, the user may be almost immediately flagged as off the route,
    /// triggering a recalculation.
    /// If this parameter is `nil`, the last location will be obtained from the configured location provider
    /// automatically.
    /// If no location is available, this method will throw an exception.
    ///   - config: Override the configuration for the navigation session. This was provided on init.
    public func startNavigation(
        route: Route,
        userLocation: UserLocation? = nil,
        config: SwiftNavigationControllerConfig? = nil
    ) throws {
        // This is technically possible, so we need to check and throw, but
        // it should be rather difficult to get a location fix, get a route,
        // and then somehow this property go nil again.
        guard let location = userLocation ?? locationProvider.lastLocation else {
            throw FerrostarCoreError.userLocationUnknown
        }
        // TODO: We should be able to circumvent this and simply start updating, wait and start nav.

        // Create the navigation session.
        let navigationSession = sessionBuilder.build(for: route, with: config?.ffiValue)

        locationProvider.startUpdating()

        self.route = route

        let navState = navigationSession.getInitialState(location: location)
        let generation = resetUpdates(to: (navigationSession, navState))
        coreNavState = navState
        state = NavigationState(
            navState: navState,
            routeGeometry: route.geometry
        )

        DispatchQueue.main.async {
            self.update(navState, location: location, generation: generation, sequence: 0)
        }
    }

    /// Resumes a previously started navigation session from the last known state.
    ///
    /// **Important! This feature is experimental and may exhibit unexpected behavior. Please
    /// report any issues you encounter to help us improve it.**
    ///
    /// - Parameter userLocation: The user's current location.
    public func resumeNavigation(
        userLocation: UserLocation? = nil
    ) throws {
        // This is technically possible, so we need to check and throw, but
        // it should be rather difficult to get a location fix, get a route,
        // and then somehow this property go nil again.
        guard let location = userLocation ?? locationProvider.lastLocation else {
            throw FerrostarCoreError.userLocationUnknown
        }
        // TODO: We should be able to circumvent this and simply start updating, wait and start nav.

        let (navigationSession, route, navState) = try sessionBuilder.buildResumedSession()

        locationProvider.startUpdating()

        self.route = route

        let generation = resetUpdates(to: (navigationSession, navState))
        coreNavState = navState
        state = NavigationState(
            navState: navState,
            routeGeometry: route.geometry
        )

        DispatchQueue.main.async {
            self.update(navState, location: location, generation: generation, sequence: 0)
        }
    }

    /// Advances to the next step. Like a location fix, this runs on the update queue, after any update already
    /// queued, and its state is published on the main thread.
    public func advanceToNextStep() {
        guard let lastLocation else {
            return
        }

        updateQueue.async { [weak self] in
            guard let self, let base = currentUpdateBase() else {
                return
            }

            let newState = base.session.advanceToNextStep(state: base.state)
            guard let sequence = commit(newState, from: base) else {
                return
            }
            update(newState, location: lastLocation, generation: base.generation, sequence: sequence)
        }
    }

    // TODO: Ability to pause without totally stopping and clearing state

    /// Stops navigation and stops requesting location updates (to save battery).
    public func stopNavigation() {
        resetUpdates(to: nil)
        route = nil
        state = nil
        queuedUtteranceIDs.removeAll()
        locationProvider.stopUpdating()
        spokenInstructionObserver.stopAndClearQueue()
        widgetProvider?.terminate()
        lastRecalculationLocation = nil
    }

    // MARK: Off-main session updates

    /// Points updates at a new session (start, resume) or at none (stop). A fix still waiting is dropped, and so is
    /// the result of an update still running for the previous session. Returns the new generation.
    @discardableResult
    private func resetUpdates(to new: (session: NavigationSession, state: NavState)?) -> UInt64 {
        updateLock.lock()
        defer { updateLock.unlock() }
        sessionGeneration &+= 1
        pendingLocation = nil
        updateBase = new.map { UpdateBase(session: $0.session, state: $0.state, generation: sessionGeneration) }
        return sessionGeneration
    }

    private func currentUpdateBase() -> UpdateBase? {
        updateLock.lock()
        defer { updateLock.unlock() }
        return updateBase
    }

    private func isCurrent(_ generation: UInt64) -> Bool {
        updateLock.lock()
        defer { updateLock.unlock() }
        return generation == sessionGeneration
    }

    /// Makes `state` the base for the next update and returns its sequence; nil when the session changed while it
    /// was computed.
    private func commit(_ state: NavState, from base: UpdateBase) -> UInt64? {
        updateLock.lock()
        defer { updateLock.unlock() }
        guard var current = updateBase, current.generation == base.generation else {
            return nil
        }
        current.state = state
        current.sequence += 1
        updateBase = current
        return current.sequence
    }

    /// Queues `location` for the session. A fix already waiting is replaced; at most one drain is queued.
    private func enqueueUpdate(_ location: UserLocation) {
        updateLock.lock()
        // Not navigating: the fix is ignored, as before.
        guard updateBase != nil else {
            updateLock.unlock()
            return
        }
        pendingLocation = location
        let schedule = !drainScheduled
        drainScheduled = true
        updateLock.unlock()

        if schedule {
            updateQueue.async { [weak self] in
                self?.drainPendingLocation()
            }
        }
    }

    /// On `updateQueue`: runs the session update for the newest waiting fix and publishes the result.
    private func drainPendingLocation() {
        updateLock.lock()
        let location = pendingLocation
        pendingLocation = nil
        drainScheduled = false
        let base = updateBase
        updateLock.unlock()
        guard let location, let base else {
            return
        }

        let newState = base.session.updateUserLocation(location: location, state: base.state)
        guard let sequence = commit(newState, from: base) else {
            return
        }
        update(newState, location: location, generation: base.generation, sequence: sequence, recordsLocation: true)
    }

    /// Internal state update. Publishes on the main thread, unless the session it came from was stopped or
    /// replaced in the meantime, or a newer state of the session was already published.
    ///
    /// You should call this rather than setting properties directly
    private func update(
        _ state: NavState,
        location: UserLocation,
        generation: UInt64,
        sequence: UInt64,
        recordsLocation: Bool = false
    ) {
        DispatchQueue.main.async {
            guard self.isCurrent(generation) else {
                return
            }
            if generation == self.publishedGeneration, sequence < self.publishedSequence {
                return
            }
            self.publishedGeneration = generation
            self.publishedSequence = sequence
            if recordsLocation {
                self.lastLocation = location
            }
            self.coreNavState = state
            self.state?.tripState = state.tripState

            switch state.tripState {
            case .idle(userLocation: _):
                break
            case let .navigating(
                currentStepGeometryIndex: _,
                userLocation: _,
                snappedUserLocation: _,
                remainingSteps: _,
                remainingWaypoints: remainingWaypoints,
                progress: tripProgress,
                summary: _,
                deviation: deviation,
                visualInstruction: visualInstruction,
                spokenInstruction: spokenInstruction,
                annotationJson: _
            ):
                switch deviation {
                case .noDeviation:
                    // No action
                    break
                case let .offRoute(deviationFromRouteLine: deviationFromRouteLine):
                    guard !self.routeRequestInFlight, // We can't have a request in flight already
                          // Ensure a minimum cool down before a new route fetch
                          self.lastAutomaticRecalculation?.timeIntervalSinceNow ?? -TimeInterval
                          .greatestFiniteMagnitude < -self
                          .minimumTimeBeforeRecalculation,
                          // Don't recalculate again if the user hasn't moved much
                          self.lastRecalculationLocation?.clLocation
                          .distance(from: location.clLocation) ?? .greatestFiniteMagnitude
                          > self
                          .minimumMovementBeforeRecalculation
                    else {
                        break
                    }

                    switch self.delegate?.core(
                        self,
                        correctiveActionForDeviation: deviationFromRouteLine,
                        remainingWaypoints: remainingWaypoints
                    ) ?? .getNewRoutes(waypoints: remainingWaypoints) {
                    case .doNothing:
                        break
                    case let .getNewRoutes(waypoints):
                        self.state?.isCalculatingNewRoute = true
                        self.lastRecalculationLocation = location
                        self.recalculationTask = Task {
                            do {
                                let routes = try await self.getRoutes(
                                    initialLocation: location,
                                    waypoints: waypoints
                                )
                                if let delegate = self.delegate {
                                    delegate.core(self, loadedAlternateRoutes: routes)
                                } else if let route = routes.first {
                                    // Default behavior when no delegate is assigned:
                                    // accept the first route, as this is what most users want when they go off route.
                                    try self.startNavigation(route: route)
                                }
                            } catch {
                                // Do nothing; this exists to enable us to run what amounts to an "async defer"
                            }

                            await MainActor.run {
                                self.lastAutomaticRecalculation = Date()
                                self.state?.isCalculatingNewRoute = false
                            }
                        }
                    }
                }

                var spokenInstructionToAlert: SpokenInstruction?
                if let spokenInstruction,
                   !self.queuedUtteranceIDs.contains(spokenInstruction.utteranceId)
                {
                    self.queuedUtteranceIDs.insert(spokenInstruction.utteranceId)

                    // Only set the spoken instruction to alert when it's queued here.
                    // Otherwise we'll ignore it.
                    spokenInstructionToAlert = spokenInstruction

                    // This should not happen on the main queue as it can block;
                    // we'll probably remove the need for this eventually
                    // by making FerrostarCore its own actor
                    DispatchQueue.global(qos: .default).async {
                        self.spokenInstructionObserver.spokenInstructionTriggered(spokenInstruction)
                    }
                }

                // Update the dynamic island if it's being used.
                if let visualInstruction {
                    self.widgetProvider?.update(
                        visualInstruction: visualInstruction,
                        spokenInstruction: spokenInstructionToAlert,
                        tripProgress: tripProgress
                    )
                }
            case .complete(userLocation: _, summary: _):
                // End the widget session if the route is completed, regardless of whether stop is called.
                // This avoids a dangling LiveActivity the user must close.
                self.widgetProvider?.terminate()
            }
        }
    }
}

extension FerrostarCore: LocationManagingDelegate {
    /// Hands the newest fix to the navigation session. The update runs off the calling thread (see the
    /// threading note on ``FerrostarCore``); its state is published on the main thread.
    public func locationManager(_: LocationProviding, didUpdateLocations locations: [UserLocation]) {
        guard let location = locations.last else {
            return
        }

        enqueueUpdate(location)
    }

    public func locationManager(_: LocationProviding, didUpdateHeading _: Heading) {
        // TODO: Make use of heading in TripState?
        //        state?.heading = newHeading
    }

    public func locationManager(_: LocationProviding, didFailWithError _: Error) {
        // TODO: Decide if/how to propagate this upstream later.
        // For initial releases, we simply assume that the developer has requested the correct permissions
        // and ensure this before attempting to start location updates.
    }
}
