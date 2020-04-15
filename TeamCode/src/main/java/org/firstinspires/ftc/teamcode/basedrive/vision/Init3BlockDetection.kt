package org.firstinspires.ftc.teamcode.basedrive.vision

import org.openftc.easyopencv.OpenCvPipeline

abstract class Init3BlockDetection : OpenCvPipeline() {
    private val stoneRowMaxWidth = 8

    var detectedSkystonePosition = -1

    val width = 640
    val height = 480

    fun getSkystonePositions(leftMostPosition: Int): Array<Int> {
        val firstSkystonePosition = detectedSkystonePosition + leftMostPosition
        var secondSkystonePosition: Int

        if(firstSkystonePosition >= stoneRowMaxWidth / 2) {
            secondSkystonePosition = firstSkystonePosition - 4
        } else {
            secondSkystonePosition = firstSkystonePosition + 4
        }

        val skystonePositions = arrayOf(firstSkystonePosition, secondSkystonePosition)
        skystonePositions.sort()

        return skystonePositions
    }
}