//! Tools for deciding when the user has sufficiently deviated from a route.
//!
//! The types in this module are designed around route deviation detection as a single responsibility:
//!
//! While the most common use for this is triggering route recalculation,
//! the decision to reroute (or display an overlay on screen, or any other action) lies with higher levels.
//!
//! For example, on iOS and Android, the `FerrostarCore` class is in charge of deciding
//! when to kick off a new route request.
//! Similarly, you may observe this in your own UI layer and display an overlay under certain conditions.
//!
//! When architecting a Ferrostar core integration for a new platform,
//! we suggest enforcing a similar separation of concerns.

use crate::algorithms::deviation_from_line;
use crate::algorithms::segment_bearings_within;
use crate::debug_eprintln;
use crate::models::{Route, RouteStep};
use crate::navigation_controller::models::TripState;
#[cfg(test)]
use crate::{models::UserLocation, navigation_controller::test_helpers::get_navigating_trip_state};
#[cfg(feature = "alloc")]
use alloc::sync::Arc;
use geo::Point;
use serde::{Deserialize, Serialize};
#[cfg(feature = "wasm-bindgen")]
use tsify::Tsify;

#[cfg(test)]
use {
    crate::{
        models::GeographicCoordinate,
        navigation_controller::test_helpers::{gen_dummy_route_step, gen_route_from_steps},
    },
    proptest::prelude::*,
};

#[cfg(all(test, feature = "std", not(feature = "web-time")))]
use std::time::SystemTime;

#[cfg(all(test, feature = "web-time"))]
use web_time::SystemTime;

/// How many remaining steps (current + upcoming) the deviation check inspects.
///
/// Matches the forward window used for puck snapping
/// (`NavigationController::build_nearby_linestring`). Large enough that a user
/// matched one or two steps ahead (short steps around a maneuver, GPS noise,
/// self-intersections) is not falsely off-route; small enough that a parallel
/// but unrelated road far ahead cannot mask a genuine deviation for long.
pub(crate) const DEVIATION_STEP_WINDOW: usize = 4;

/// Determines if the user has deviated from the expected route.
#[derive(Clone, Serialize, Deserialize)]
#[cfg_attr(feature = "uniffi", derive(uniffi::Enum))]
#[cfg_attr(feature = "wasm-bindgen", derive(Tsify))]
#[cfg_attr(feature = "wasm-bindgen", tsify(from_wasm_abi))]
pub enum RouteDeviationTracking {
    /// No checks will be done, and we assume the user is always following the route.
    None,
    /// Detects deviation from the route using a configurable static distance threshold from the route line.
    ///
    /// The distance is measured against a short forward window of steps
    /// ([`DEVIATION_STEP_WINDOW`]): the user is off-route only when farther
    /// than the threshold from EVERY step in the window. Checking the current
    /// step alone reported false off-route whenever the user was matched
    /// slightly ahead of it.
    #[cfg_attr(feature = "wasm-bindgen", serde(rename_all = "camelCase"))]
    StaticThreshold {
        /// The minimum required horizontal accuracy of the user location, in meters.
        /// Values larger than this will not trigger route deviation warnings.
        minimum_horizontal_accuracy: u16,
        /// The maximum acceptable deviation from the route line, in meters.
        ///
        /// If the distance between the reported location and the expected route line
        /// is greater than this threshold, it will be flagged as an off route condition.
        max_acceptable_deviation: f64,
    },
    /// Detects deviation using distance from a short forward window of steps,
    /// with an additional heading check to catch wrong-direction travel.
    ///
    /// Distance: the user is off-route when they are farther than the
    /// threshold from EVERY step in the window (current + the next few).
    /// Checking only the current step reported false off-route whenever the
    /// user was matched slightly ahead (short steps, GPS noise at a maneuver,
    /// self-intersecting geometry).
    ///
    /// Heading: when moving fast enough, the user is flagged off-route if
    /// their course disagrees with EVERY window segment they are plausibly on.
    /// On overlapping geometry (out-and-back roads, U-turn bridges) several
    /// directions are legitimate at once and alignment with any of them
    /// counts as on-course.
    #[cfg_attr(feature = "wasm-bindgen", serde(rename_all = "camelCase"))]
    StaticThresholdWithHeading {
        /// The minimum required horizontal accuracy of the user location, in meters.
        /// Values larger than this will not trigger route deviation warnings.
        minimum_horizontal_accuracy: u16,
        /// The maximum acceptable deviation from the route line, in meters.
        max_acceptable_deviation: f64,
        /// If the angle between the user's heading and the nearest segment bearing
        /// is greater than or equal to this value (in degrees), the user is flagged off-route.
        /// A typical value is 90.0 to 120.0 degrees.
        max_heading_deviation_degrees: f64,
        /// The minimum speed (in m/s) required before the heading check is applied.
        /// Below this speed, only the distance check is used.
        /// This avoids false positives when the user is stationary or moving slowly
        /// (where course_over_ground is unreliable).
        min_speed_for_heading_check: f64,
    },
    // TODO: Standard variants that account for mode of travel. For example, `DefaultFor(modeOfTravel: ModeOfTravel)` with sensible defaults for walking, driving, cycling, etc.
    /// An arbitrary user-defined implementation.
    /// You decide with your own [`RouteDeviationDetector`] implementation!
    #[serde(skip)]
    Custom {
        detector: Arc<dyn RouteDeviationDetector>,
    },
}

impl RouteDeviationTracking {
    /// Returns the max acceptable deviation distance (in meters) if configured, or `None`.
    ///
    /// This is used by the navigation controller to guard speed-run step advancement:
    /// if the user would be farther than this distance from the new step, the advance is rejected.
    #[must_use]
    pub(crate) fn max_deviation_distance(&self) -> Option<f64> {
        match self {
            RouteDeviationTracking::None | RouteDeviationTracking::Custom { .. } => None,
            RouteDeviationTracking::StaticThreshold {
                max_acceptable_deviation,
                ..
            } => Some(*max_acceptable_deviation),
            RouteDeviationTracking::StaticThresholdWithHeading {
                max_acceptable_deviation,
                ..
            } => Some(*max_acceptable_deviation),
        }
    }

    #[must_use]
    pub(crate) fn check_route_deviation(
        &self,
        route: &Route,
        trip_state: &TripState,
    ) -> RouteDeviation {
        match self {
            RouteDeviationTracking::None => RouteDeviation::NoDeviation,
            RouteDeviationTracking::StaticThreshold {
                minimum_horizontal_accuracy,
                max_acceptable_deviation,
            } => match trip_state {
                TripState::Idle { .. } | TripState::Complete { .. } => RouteDeviation::NoDeviation,
                TripState::Navigating {
                    user_location,
                    remaining_steps,
                    ..
                } => {
                    if user_location.horizontal_accuracy > f64::from(*minimum_horizontal_accuracy) {
                        return RouteDeviation::NoDeviation;
                    }

                    let user_pt = Point::from(*user_location);
                    match Self::min_deviation_over_step_window(&user_pt, remaining_steps) {
                        Some(deviation)
                            if deviation > 0.0 && deviation > *max_acceptable_deviation =>
                        {
                            RouteDeviation::OffRoute {
                                deviation_from_route_line: deviation,
                            }
                        }
                        _ => RouteDeviation::NoDeviation,
                    }
                }
            },
            RouteDeviationTracking::Custom { detector } => {
                detector.check_route_deviation(route.clone(), trip_state.clone())
            }
            RouteDeviationTracking::StaticThresholdWithHeading {
                minimum_horizontal_accuracy,
                max_acceptable_deviation,
                max_heading_deviation_degrees,
                min_speed_for_heading_check,
            } => match trip_state {
                TripState::Idle { .. } | TripState::Complete { .. } => RouteDeviation::NoDeviation,
                TripState::Navigating {
                    user_location,
                    remaining_steps,
                    ..
                } => {
                    // Accuracy gate
                    if user_location.horizontal_accuracy > f64::from(*minimum_horizontal_accuracy) {
                        debug_eprintln!(
                            "[HeadingCheck] SKIP: accuracy {:.1} > threshold {}",
                            user_location.horizontal_accuracy,
                            minimum_horizontal_accuracy
                        );
                        return RouteDeviation::NoDeviation;
                    }

                    let user_pt = Point::from(*user_location);

                    let Some(deviation_m) =
                        Self::min_deviation_over_step_window(&user_pt, remaining_steps)
                    else {
                        debug_eprintln!("[HeadingCheck] SKIP: no usable step geometry in window");
                        return RouteDeviation::NoDeviation;
                    };

                    if deviation_m > *max_acceptable_deviation {
                        debug_eprintln!(
                            "[HeadingCheck] OFF_ROUTE by distance: {:.2}m",
                            deviation_m
                        );
                        return RouteDeviation::OffRoute {
                            deviation_from_route_line: deviation_m,
                        };
                    }

                    // Heading check: only when moving fast enough and course is available.
                    let speed_mps = user_location.speed.map(|s| s.value).unwrap_or(0.0);
                    if speed_mps >= *min_speed_for_heading_check {
                        if let Some(cog) = user_location.course_over_ground {
                            let user_heading = normalize_deg(cog.degrees as f64);

                            // Bearings of EVERY window segment the user is
                            // plausibly on (within the distance tolerance).
                            // Overlapping geometry puts several directions
                            // under the user at once; only disagreement with
                            // ALL of them is wrong-direction travel.
                            let mut candidate_bearings: Vec<f64> = Vec::new();
                            for step in remaining_steps.iter().take(DEVIATION_STEP_WINDOW) {
                                let step_ls = step.get_linestring();
                                if step_ls.0.len() < 2 {
                                    continue;
                                }
                                candidate_bearings.extend(segment_bearings_within(
                                    &user_pt,
                                    &step_ls,
                                    *max_acceptable_deviation,
                                ));
                            }

                            let misaligned_with_all = !candidate_bearings.is_empty()
                                && candidate_bearings.iter().all(|bearing| {
                                    smallest_angle_diff_deg(user_heading, normalize_deg(*bearing))
                                        >= *max_heading_deviation_degrees
                                });
                            if misaligned_with_all {
                                debug_eprintln!(
                                    "[HeadingCheck] OFF_ROUTE by heading: heading={:.1}° disagrees with all {} nearby segments",
                                    user_heading,
                                    candidate_bearings.len()
                                );
                                // The 1 m floor marks heading trips until a
                                // structured deviation kind is exposed over
                                // FFI; the true perpendicular distance here is
                                // near zero and downstream treats the value as
                                // informational.
                                return RouteDeviation::OffRoute {
                                    deviation_from_route_line: deviation_m.max(1.0),
                                };
                            }
                        } else {
                            debug_eprintln!("[HeadingCheck] SKIP heading: no course_over_ground");
                        }
                    } else {
                        debug_eprintln!(
                            "[HeadingCheck] SKIP heading: speed too low ({:.2} < {:.1})",
                            speed_mps,
                            min_speed_for_heading_check
                        );
                    }

                    RouteDeviation::NoDeviation
                }
            },
        }
    }

    /// Minimum distance (in meters) from the user to any step in the forward
    /// deviation window ([`DEVIATION_STEP_WINDOW`]).
    ///
    /// Returns `None` when no step in the window has usable geometry.
    fn min_deviation_over_step_window(
        user_pt: &Point,
        remaining_steps: &[RouteStep],
    ) -> Option<f64> {
        let mut min_deviation: Option<f64> = None;
        for step in remaining_steps.iter().take(DEVIATION_STEP_WINDOW) {
            let step_ls = step.get_linestring();
            if step_ls.0.len() < 2 {
                continue;
            }
            if let Some(d) = deviation_from_line(user_pt, &step_ls) {
                if min_deviation.map_or(true, |m| d < m) {
                    min_deviation = Some(d);
                }
            }
        }
        min_deviation
    }
}

#[inline]
fn normalize_deg(x: f64) -> f64 {
    ((x % 360.0) + 360.0) % 360.0
}

#[inline]
fn smallest_angle_diff_deg(a: f64, b: f64) -> f64 {
    let d = (a - b + 540.0) % 360.0 - 180.0;
    d.abs()
}

/// Status information that describes whether the user is proceeding according to the route or not.
///
/// Note that the name is intentionally a bit generic to allow for expansion of other states.
/// For example, we could conceivably add a "wrong way" status in the future.
#[derive(Debug, Copy, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "uniffi", derive(uniffi::Enum))]
#[cfg_attr(feature = "wasm-bindgen", derive(Tsify))]
#[cfg_attr(feature = "wasm-bindgen", tsify(into_wasm_abi, from_wasm_abi))]
pub enum RouteDeviation {
    /// The user is proceeding on course within the expected tolerances; everything is normal.
    NoDeviation,
    /// The user is off the expected route.
    #[cfg_attr(feature = "wasm-bindgen", serde(rename_all = "camelCase"))]
    OffRoute {
        /// The deviation from the route line, in meters.
        deviation_from_route_line: f64,
    },
}

/// A custom deviation detector (for extending the behavior of [`RouteDeviationTracking`]).
///
/// This allows for arbitrarily complex implementations when the provided ones are not enough.
/// For example, detecting that the user is proceeding the wrong direction by keeping a ring buffer
/// of recent locations, or perform local map matching.
#[cfg_attr(feature = "uniffi", uniffi::export(with_foreign))]
pub trait RouteDeviationDetector: Send + Sync {
    /// Determines whether the user is following the route correctly or not.
    ///
    /// NOTE: This function has a single responsibility.
    /// Side-effects like whether to recalculate a route are left to higher levels,
    /// and implementations should only be concerned with determining the facts.
    ///
    /// IMPORTANT: If you are short circuiting [`StepAdvanceCondition`]'s to allow
    /// skipping steps, you must always fall back to checking the deviation from the
    /// full route line.
    #[must_use]
    fn check_route_deviation(&self, route: Route, trip_state: TripState) -> RouteDeviation;
}

#[cfg(test)]
proptest! {
    /// Tests [`RouteDeviationTracking::None`] behavior,
    /// which never reports that the user is off route, even when they obviously are.
    #[test]
    fn no_deviation_tracking(
        x1: f64, y1: f64,
        x2: f64, y2: f64,
        x3: f64, y3: f64,
    ) {
        let tracking = RouteDeviationTracking::None;
        let current_route_step = gen_dummy_route_step(x1, y1, x2, y2);
        let route = gen_route_from_steps(vec![current_route_step.clone()]);

        // Set the user location to the start of the route step.
        // This is clearly on the route.
        let user_location_on_route = UserLocation {
            coordinates: GeographicCoordinate {
                lng: x1,
                lat: y1,
            },
            horizontal_accuracy: 0.0,
            course_over_ground: None,
            timestamp: SystemTime::now(),
            speed: None,
            altitude: None,
            vertical_accuracy: None,
        };
        let trip_state = get_navigating_trip_state(
            user_location_on_route.clone(),
            vec![current_route_step.clone()],
            vec![],
            RouteDeviation::NoDeviation
        );
        prop_assert_eq!(
            tracking.check_route_deviation(&route, &trip_state),
            RouteDeviation::NoDeviation
        );

        // Set the user location to a random value.
        // This may be well off route, but we don't care in this mode.
        let user_location_random = UserLocation {
            coordinates: GeographicCoordinate {
                lng: x3,
                lat: y3,
            },
            horizontal_accuracy: 0.0,
            course_over_ground: None,
            timestamp: SystemTime::now(),
            speed: None,
            altitude: None,
            vertical_accuracy: None,
        };
        let trip_state_random = get_navigating_trip_state(
            user_location_random.clone(),
            vec![current_route_step.clone()],
            vec![],
            RouteDeviation::NoDeviation
        );
        prop_assert_eq!(
            tracking.check_route_deviation(&route, &trip_state_random),
            RouteDeviation::NoDeviation
        );
    }

    /// Implements the same behavior as [`RouteDeviationTracking::None`]
    /// with user-supplied code.
    #[test]
    fn custom_no_deviation_mode(
        x1: f64, y1: f64,
        x2: f64, y2: f64,
        x3: f64, y3: f64,
    ) {
        struct NeverDetector {}

        impl RouteDeviationDetector for NeverDetector {
            fn check_route_deviation(
                &self,
                _route: Route,
                _trip_state: TripState,
            ) -> RouteDeviation {
                return RouteDeviation::NoDeviation
            }
        }

        let tracking = RouteDeviationTracking::Custom {
            detector: Arc::new(NeverDetector {})
        };
        let current_route_step = gen_dummy_route_step(x1, y1, x2, y2);
        let route = gen_route_from_steps(vec![current_route_step.clone()]);

        // Set the user location to the start of the route step.
        // This is clearly on the route.
        let user_location_on_route = UserLocation {
            coordinates: GeographicCoordinate {
                lng: x1,
                lat: y1,
            },
            horizontal_accuracy: 0.0,
            course_over_ground: None,
            timestamp: SystemTime::now(),
            speed: None,
            altitude: None,
            vertical_accuracy: None,
        };
        let trip_state_on_route = get_navigating_trip_state(
            user_location_on_route.clone(),
            vec![current_route_step.clone()],
            vec![],
            RouteDeviation::NoDeviation
        );
        prop_assert_eq!(
            tracking.check_route_deviation(&route, &trip_state_on_route),
            RouteDeviation::NoDeviation
        );

        // Set the user location to a random value.
        // This may be well off route, but we don't care in this mode.
        let user_location_random = UserLocation {
            coordinates: GeographicCoordinate {
                lng: x3,
                lat: y3,
            },
            horizontal_accuracy: 0.0,
            course_over_ground: None,
            timestamp: SystemTime::now(),
            speed: None,
            altitude: None,
            vertical_accuracy: None,
        };
        let trip_state_random = get_navigating_trip_state(
            user_location_random.clone(),
            vec![current_route_step.clone()],
            vec![],
            RouteDeviation::NoDeviation
        );
        prop_assert_eq!(
            tracking.check_route_deviation(&route, &trip_state_random),
            RouteDeviation::NoDeviation
        );
    }

    /// Custom behavior claiming that the user is always off the route.
    #[test]
    fn custom_always_off_route(
        x1: f64, y1: f64,
        x2: f64, y2: f64,
        x3: f64, y3: f64,
    ) {
        struct NeverDetector {}

        impl RouteDeviationDetector for NeverDetector {
            fn check_route_deviation(
                &self,
                _route: Route,
                _trip_state: TripState,
            ) -> RouteDeviation {
                return RouteDeviation::OffRoute {
                    deviation_from_route_line: 7.0
                }
            }
        }

        let tracking = RouteDeviationTracking::Custom {
            detector: Arc::new(NeverDetector {})
        };
        let current_route_step = gen_dummy_route_step(x1, y1, x2, y2);
        let route = gen_route_from_steps(vec![current_route_step.clone()]);

        // Set the user location to the start of the route step.
        // This is clearly on the route.
        let user_location_on_route = UserLocation {
            coordinates: GeographicCoordinate {
                lng: x1,
                lat: y1,
            },
            horizontal_accuracy: 0.0,
            course_over_ground: None,
            timestamp: SystemTime::now(),
            speed: None,
            altitude: None,
            vertical_accuracy: None,
        };
        let trip_state_on_route = get_navigating_trip_state(
            user_location_on_route.clone(),
            vec![current_route_step.clone()],
            vec![],
            RouteDeviation::NoDeviation
        );
        prop_assert_eq!(
            tracking.check_route_deviation(&route, &trip_state_on_route),
            RouteDeviation::OffRoute {
                deviation_from_route_line: 7.0
            }
        );

        // Set the user location to a random value.
        // This may be well off route, but we don't care in this mode.
        let user_location_random = UserLocation {
            coordinates: GeographicCoordinate {
                lng: x3,
                lat: y3,
            },
            horizontal_accuracy: 0.0,
            course_over_ground: None,
            timestamp: SystemTime::now(),
            speed: None,
            altitude: None,
            vertical_accuracy: None,
        };
        let trip_state_random = get_navigating_trip_state(
            user_location_random.clone(),
            vec![current_route_step.clone()],
            vec![],
            RouteDeviation::NoDeviation
        );
        prop_assert_eq!(
            tracking.check_route_deviation(&route, &trip_state_random),
            RouteDeviation::OffRoute {
                deviation_from_route_line: 7.0
            }
        );
    }

    /// Tests [`RouteDeviationTracking::StaticThreshold`] behavior,
    /// using [`algorithms::deviation_from_line`](crate::algorithms::deviation_from_line)
    #[test]
    fn static_threshold_oracle_test(
        x1: f64, y1: f64,
        x2: f64, y2: f64,
        x3: f64, y3: f64,
        minimum_horizontal_accuracy: u16,
        horizontal_accuracy: f64,
        max_acceptable_deviation in 0f64..,
    ) {
        let tracking = RouteDeviationTracking::StaticThreshold {
            minimum_horizontal_accuracy,
            max_acceptable_deviation
        };
        let current_route_step = gen_dummy_route_step(x1, y1, x2, y2);
        let route = gen_route_from_steps(vec![current_route_step.clone()]);

        // Set the user location to the start of the route step.
        // This is clearly on the route.
        let user_location_on_route = UserLocation {
            coordinates: GeographicCoordinate {
                lng: x1,
                lat: y1,
            },
            horizontal_accuracy,
            course_over_ground: None,
            timestamp: SystemTime::now(),
            speed: None,
            altitude: None,
            vertical_accuracy: None,
        };
        let trip_state = get_navigating_trip_state(
            user_location_on_route.clone(),
            vec![current_route_step.clone()],
            vec![],
            RouteDeviation::NoDeviation
        );
        prop_assert_eq!(
            tracking.check_route_deviation(&route, &trip_state),
            RouteDeviation::NoDeviation
        );

        // Set the user location to a random value.
        // This may be well off route. Check the deviation_from_line helper
        // as an oracle.
        let coordinates = GeographicCoordinate {
            lng: x3,
            lat: y3,
        };
        let user_location_random = UserLocation {
            coordinates,
            horizontal_accuracy: 0.0,
            course_over_ground: None,
            timestamp: SystemTime::now(),
            speed: None,
            altitude: None,
            vertical_accuracy: None,
        };
        let trip_state_random = get_navigating_trip_state(
            user_location_random.clone(),
            vec![current_route_step.clone()],
            vec![],
            RouteDeviation::NoDeviation
        );
        let deviation = deviation_from_line(&Point::from(coordinates), &current_route_step.get_linestring());
        match tracking.check_route_deviation(&route, &trip_state_random) {
            RouteDeviation::NoDeviation => {
                if let Some(calculated) = deviation {
                    prop_assert!(calculated <= max_acceptable_deviation);
                }
            }
            RouteDeviation::OffRoute{ deviation_from_route_line } => {
                prop_assert_eq!(
                    deviation_from_route_line,
                    deviation.unwrap()
                );
            }
        }
    }

    /// Tests [`RouteDeviationTracking::StaticThresholdWithHeading`] behavior
    /// when the user is traveling backward on the step line.
    /// The user is near the step line (deviation is small) but heading in the opposite direction.
    #[test]
    fn heading_check_detects_backward_travel(
        // Use a step going roughly east (lng increases, lat constant)
        // User is on the line but heading west (bearing ~270)
        x1 in -170f64..170f64,
        y1 in -80f64..80f64,
    ) {
        let x2 = x1 + 0.01; // ~1 km east
        let y2 = y1;         // same latitude = due east

        let tracking = RouteDeviationTracking::StaticThresholdWithHeading {
            minimum_horizontal_accuracy: 15,
            max_acceptable_deviation: 50.0,
            max_heading_deviation_degrees: 110.0,
            min_speed_for_heading_check: 2.0,
        };

        let current_route_step = gen_dummy_route_step(x1, y1, x2, y2);
        let route = gen_route_from_steps(vec![current_route_step.clone()]);

        // User is at the midpoint of the step (clearly on the line)
        // but heading west (270 degrees) — opposite of step direction (~90 degrees east)
        let midpoint_lng = (x1 + x2) / 2.0;
        let midpoint_lat = (y1 + y2) / 2.0;
        let user_location = UserLocation {
            coordinates: GeographicCoordinate {
                lng: midpoint_lng,
                lat: midpoint_lat,
            },
            horizontal_accuracy: 5.0,
            course_over_ground: Some(crate::models::CourseOverGround { degrees: 270, accuracy: None }),
            timestamp: SystemTime::now(),
            speed: Some(crate::models::Speed { value: 10.0, accuracy: None }),
            altitude: None,
            vertical_accuracy: None,
        };

        let trip_state = get_navigating_trip_state(
            user_location,
            vec![current_route_step.clone()],
            vec![],
            RouteDeviation::NoDeviation,
        );

        let result = tracking.check_route_deviation(&route, &trip_state);

        // The user is on the line, so deviation_m is small.
        // But heading is ~180 degrees off — should be flagged off-route.
        match result {
            RouteDeviation::OffRoute { .. } => { /* expected */ }
            RouteDeviation::NoDeviation => {
                // Debug: compute the values manually to understand the failure
                let user_pt = Point::new(midpoint_lng, midpoint_lat);
                let step_ls = current_route_step.get_linestring();
                let dev = deviation_from_line(&user_pt, &step_ls);
                let seg_brg = crate::algorithms::nearest_segment_bearing_deg(&user_pt, &step_ls);
                let user_heading = normalize_deg(270.0);
                let delta = seg_brg.map(|b| smallest_angle_diff_deg(user_heading, normalize_deg(b)));
                prop_assert!(
                    false,
                    "Expected OffRoute but got NoDeviation. deviation_m={:?}, seg_bearing={:?}, user_heading={}, delta={:?}",
                    dev, seg_brg, user_heading, delta
                );
            }
        }
    }

    /// Tests [`RouteDeviationTracking::StaticThresholdWithHeading`] behavior
    /// when the user is traveling forward — should NOT flag off-route.
    #[test]
    fn heading_check_allows_forward_travel(
        x1 in -170f64..170f64,
        y1 in -80f64..80f64,
    ) {
        let x2 = x1 + 0.01;
        let y2 = y1;

        let tracking = RouteDeviationTracking::StaticThresholdWithHeading {
            minimum_horizontal_accuracy: 15,
            max_acceptable_deviation: 50.0,
            max_heading_deviation_degrees: 110.0,
            min_speed_for_heading_check: 2.0,
        };

        let current_route_step = gen_dummy_route_step(x1, y1, x2, y2);
        let route = gen_route_from_steps(vec![current_route_step.clone()]);

        let midpoint_lng = (x1 + x2) / 2.0;
        let midpoint_lat = (y1 + y2) / 2.0;
        let user_location = UserLocation {
            coordinates: GeographicCoordinate {
                lng: midpoint_lng,
                lat: midpoint_lat,
            },
            horizontal_accuracy: 5.0,
            // Heading east (~90) matching the step direction
            course_over_ground: Some(crate::models::CourseOverGround { degrees: 90, accuracy: None }),
            timestamp: SystemTime::now(),
            speed: Some(crate::models::Speed { value: 10.0, accuracy: None }),
            altitude: None,
            vertical_accuracy: None,
        };

        let trip_state = get_navigating_trip_state(
            user_location,
            vec![current_route_step.clone()],
            vec![],
            RouteDeviation::NoDeviation,
        );

        let result = tracking.check_route_deviation(&route, &trip_state);
        prop_assert_eq!(result, RouteDeviation::NoDeviation);
    }

    /// Tests [`RouteDeviationTracking::StaticThreshold`] behavior
    /// for values which are not accurate enough.
    #[test]
    fn static_threshold_ignores_inaccurate_location_updates(
        x1 in -180f64..=180f64, y1 in -90f64..=90f64,
        x2 in -180f64..=180f64, y2 in -90f64..=90f64,
        x3 in -180f64..=180f64, y3 in -90f64..=90f64,
        horizontal_accuracy in 1u16..,
        max_acceptable_deviation: f64,
    ) {
        let tracking = RouteDeviationTracking::StaticThreshold {
            minimum_horizontal_accuracy: horizontal_accuracy - 1,
            max_acceptable_deviation
        };
        let current_route_step = gen_dummy_route_step(x1, y1, x2, y2);
        let route = gen_route_from_steps(vec![current_route_step.clone()]);

        let coordinates = GeographicCoordinate {
            lng: x3,
            lat: y3,
        };
        let user_location_random = UserLocation {
            coordinates,
            horizontal_accuracy: horizontal_accuracy as f64,
            course_over_ground: None,
            timestamp: SystemTime::now(),
            speed: None,
            altitude: None,
            vertical_accuracy: None,
        };
        let trip_state_random = get_navigating_trip_state(
            user_location_random.clone(),
            vec![current_route_step.clone()],
            vec![],
            RouteDeviation::NoDeviation
        );
        prop_assert_eq!(
            tracking.check_route_deviation(&route, &trip_state_random),
            RouteDeviation::NoDeviation
        );
    }
}

#[cfg(test)]
mod window_and_heading_tests {
    use super::*;
    use crate::models::{CourseOverGround, GeographicCoordinate, Speed, UserLocation};
    use crate::navigation_controller::test_helpers::{
        gen_dummy_route_step, gen_route_from_steps, gen_route_step_with_coords,
        get_navigating_trip_state,
    };
    use geo::coord;

    fn tracking() -> RouteDeviationTracking {
        // The Stegra production configuration.
        RouteDeviationTracking::StaticThresholdWithHeading {
            minimum_horizontal_accuracy: 15,
            max_acceptable_deviation: 50.0,
            max_heading_deviation_degrees: 110.0,
            min_speed_for_heading_check: 2.0,
        }
    }

    fn location(
        lng: f64,
        lat: f64,
        course_deg: Option<u16>,
        speed_mps: Option<f64>,
    ) -> UserLocation {
        UserLocation {
            coordinates: GeographicCoordinate { lng, lat },
            horizontal_accuracy: 5.0,
            course_over_ground: course_deg.map(|degrees| CourseOverGround {
                degrees,
                accuracy: None,
            }),
            timestamp: SystemTime::now(),
            speed: speed_mps.map(|value| Speed {
                value,
                accuracy: None,
            }),
            altitude: None,
            vertical_accuracy: None,
        }
    }

    /// A user physically on the NEXT step must not be off-route just because
    /// the current step is far behind them (short steps, GPS matched ahead).
    #[test]
    fn window_allows_user_on_future_step() {
        let step0 = gen_dummy_route_step(0.0, 0.0, 0.005, 0.0);
        let step1 = gen_dummy_route_step(0.005, 0.0, 0.01, 0.0);
        let route = gen_route_from_steps(vec![step0.clone(), step1.clone()]);

        // ~278 m past step0's end, ~11 m from step1's line.
        let user = location(0.0075, 0.0001, None, None);
        let trip_state = get_navigating_trip_state(
            user,
            vec![step0, step1],
            vec![],
            RouteDeviation::NoDeviation,
        );

        assert_eq!(
            tracking().check_route_deviation(&route, &trip_state),
            RouteDeviation::NoDeviation
        );
    }

    /// A user far from every window step is still flagged, and the reported
    /// deviation is the true distance to the nearest window step.
    #[test]
    fn window_flags_genuine_deviation_with_true_distance() {
        let step0 = gen_dummy_route_step(0.0, 0.0, 0.005, 0.0);
        let step1 = gen_dummy_route_step(0.005, 0.0, 0.01, 0.0);
        let route = gen_route_from_steps(vec![step0.clone(), step1.clone()]);

        // ~550 m north of the whole route.
        let user = location(0.0075, 0.005, None, None);
        let trip_state = get_navigating_trip_state(
            user,
            vec![step0, step1],
            vec![],
            RouteDeviation::NoDeviation,
        );

        match tracking().check_route_deviation(&route, &trip_state) {
            RouteDeviation::OffRoute {
                deviation_from_route_line,
            } => {
                assert!(
                    deviation_from_route_line > 500.0,
                    "expected the true distance (~550 m), got {deviation_from_route_line:.1}"
                );
            }
            RouteDeviation::NoDeviation => panic!("expected OffRoute"),
        }
    }

    /// On an out-and-back (anti-parallel legs within GPS noise of each other),
    /// travel in EITHER direction counts as on-course — comparing against only
    /// the single nearest segment made this flip arbitrarily.
    #[test]
    fn heading_allows_both_directions_on_overlapping_legs() {
        let step = gen_route_step_with_coords(vec![
            coord! { x: 0.0, y: 0.0 },
            coord! { x: 0.004, y: 0.0 },
            coord! { x: 0.004, y: 0.0002 },
            coord! { x: 0.0, y: 0.0002 },
        ]);
        let route = gen_route_from_steps(vec![step.clone()]);

        for course in [90u16, 270u16] {
            let user = location(0.002, 0.0001, Some(course), Some(10.0));
            let trip_state = get_navigating_trip_state(
                user,
                vec![step.clone()],
                vec![],
                RouteDeviation::NoDeviation,
            );
            assert_eq!(
                tracking().check_route_deviation(&route, &trip_state),
                RouteDeviation::NoDeviation,
                "course {course}° should be on-course between anti-parallel legs"
            );
        }
    }

    /// On a plain one-way line, wrong-direction travel is still flagged.
    #[test]
    fn heading_flags_wrong_way_on_simple_line() {
        let step = gen_dummy_route_step(0.0, 0.0, 0.004, 0.0);
        let route = gen_route_from_steps(vec![step.clone()]);

        let user = location(0.002, 0.0, Some(270), Some(10.0));
        let trip_state =
            get_navigating_trip_state(user, vec![step], vec![], RouteDeviation::NoDeviation);

        match tracking().check_route_deviation(&route, &trip_state) {
            RouteDeviation::OffRoute {
                deviation_from_route_line,
            } => {
                // Heading trips report the 1 m floor (true perpendicular
                // distance is ~0 here).
                assert!(deviation_from_route_line >= 1.0);
            }
            RouteDeviation::NoDeviation => panic!("expected wrong-way OffRoute"),
        }
    }
}
