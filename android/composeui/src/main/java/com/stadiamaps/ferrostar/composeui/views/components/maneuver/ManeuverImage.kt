package com.stadiamaps.ferrostar.composeui.views.components.maneuver

import android.annotation.SuppressLint
import androidx.compose.foundation.layout.size
import androidx.compose.material3.Icon
import androidx.compose.material3.LocalContentColor
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import com.stadiamaps.ferrostar.composeui.R
import uniffi.ferrostar.ManeuverModifier
import uniffi.ferrostar.ManeuverType
import uniffi.ferrostar.VisualInstructionContent

private fun roundaboutIconFromDegrees(degrees: UShort?): String {
  if (degrees == null) return "direction_roundabout"

  // Convert safely to Int
  val d = (degrees.toInt() % 360 + 360) % 360

  val isLeft = d > 180
  val angleFromAxis = if (isLeft) 360 - d else d // 0..180 from straight ahead

  // ktfmt forbids vertical alignment of arrows
  val suffix =
      when {
        angleFromAxis < 15 -> "straight"
        angleFromAxis < 45 -> if (isLeft) "sharp_left" else "sharp_right"
        angleFromAxis < 90 -> if (isLeft) "left" else "right"
        else -> if (isLeft) "slight_left" else "slight_right"
      }

  return "direction_roundabout_$suffix"
}

val VisualInstructionContent.maneuverIcon: String
  get() {
    val typeName = maneuverType?.name?.lowercase()

    // Handle your custom "Bird" type here!
    if (typeName == "bird") {
      return "direction_bird" // Ensure you have a drawable named 'direction_bird'
    }

    val isRoundabout =
        when (typeName) {
          "roundabout",
          "exit_roundabout",
          "rotary" -> true
          else -> false
        }
    if (isRoundabout) {
      return roundaboutIconFromDegrees(roundaboutExitDegrees)
    }

    val descriptor =
        listOfNotNull(
                maneuverType?.name?.replace(" ", "_"), maneuverModifier?.name?.replace(" ", "_"))
            .joinToString(separator = "_")

    return "direction_${descriptor}".lowercase()
  }

/** An icon view using the public domain drawables from Mapbox. */
@SuppressLint("DiscouragedApi")
@Composable
fun ManeuverImage(content: VisualInstructionContent, tint: Color = LocalContentColor.current) {
  val context = LocalContext.current
  val resourceId =
      context.resources.getIdentifier(content.maneuverIcon, "drawable", context.packageName)

  if (resourceId != 0) {
    Icon(
        painter = painterResource(id = resourceId),
        contentDescription = stringResource(id = R.string.maneuver_image),
        tint = tint,
        modifier = Modifier.size(64.dp))
  } else {
    // Ignore resolution failures for the moment.
  }
}

@Preview
@Composable
fun ManeuverImageLeftTurnPreview() {
  ManeuverImage(
      VisualInstructionContent(
          text = "",
          maneuverType = ManeuverType.TURN,
          maneuverModifier = ManeuverModifier.LEFT,
          roundaboutExitDegrees = null,
          laneInfo = null,
          exitNumbers = emptyList()))
}

@Preview
@Composable
fun ManeuverImageContinueUturnPreview() {
  ManeuverImage(
      VisualInstructionContent(
          text = "",
          maneuverType = ManeuverType.CONTINUE,
          maneuverModifier = ManeuverModifier.U_TURN,
          roundaboutExitDegrees = null,
          laneInfo = null,
          exitNumbers = emptyList()))
}
