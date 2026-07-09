import CoreLocation
import FerrostarCoreFFI
import Foundation

private final class DetectorImpl: RouteDeviationDetector {
    let detectorFunc: @Sendable (Route, TripState) -> RouteDeviation

    init(detectorFunc: @escaping @Sendable (Route, TripState) -> RouteDeviation) {
        self.detectorFunc = detectorFunc
    }

    func checkRouteDeviation(route: Route, tripState: TripState) -> RouteDeviation {
        detectorFunc(route, tripState)
    }
}

/// A Swift wrapper around `UniFFI.RouteDeviationTracking`
public enum SwiftRouteDeviationTracking {
    case none

    case staticThreshold(minimumHorizontalAccuracy: UInt16, maxAcceptableDeviation: Double)

    case staticThresholdWithHeading(
        minimumHorizontalAccuracy: UInt16,
        maxAcceptableDeviation: Double,
        maxHeadingDeviationDegrees: Double,
        minSpeedForHeadingCheck: Double
    )

    case custom(detector: @Sendable (Route, TripState) -> RouteDeviation)

    var ffiValue: FerrostarCoreFFI.RouteDeviationTracking {
        switch self {
        case .none:
            .none
        case let .staticThreshold(
            minimumHorizontalAccuracy: minimumHorizontalAccuracy,
            maxAcceptableDeviation: maxAcceptableDeviation
        ):
            .staticThreshold(
                minimumHorizontalAccuracy: minimumHorizontalAccuracy,
                maxAcceptableDeviation: maxAcceptableDeviation
            )
        case let .staticThresholdWithHeading(
            minimumHorizontalAccuracy: minimumHorizontalAccuracy,
            maxAcceptableDeviation: maxAcceptableDeviation,
            maxHeadingDeviationDegrees: maxHeadingDeviationDegrees,
            minSpeedForHeadingCheck: minSpeedForHeadingCheck
        ):
            .staticThresholdWithHeading(
                minimumHorizontalAccuracy: minimumHorizontalAccuracy,
                maxAcceptableDeviation: maxAcceptableDeviation,
                maxHeadingDeviationDegrees: maxHeadingDeviationDegrees,
                minSpeedForHeadingCheck: minSpeedForHeadingCheck
            )
        case let .custom(detector: detectorFunc):
            .custom(detector: DetectorImpl(detectorFunc: detectorFunc))
        }
    }
}
