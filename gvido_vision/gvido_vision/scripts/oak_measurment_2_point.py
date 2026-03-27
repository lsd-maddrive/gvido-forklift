#!/usr/bin/env python3

import cv2
import depthai as dai
import numpy as np

color = (255, 255, 255)

center_x = 0.5        
center_y = 0.5
point_half_size = 0.004 

# Create pipeline
pipeline = dai.Pipeline()

monoLeft = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
monoRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
stereo = pipeline.create(dai.node.StereoDepth)
spatialLocationCalculator = pipeline.create(dai.node.SpatialLocationCalculator)

# Linking
monoLeftOut = monoLeft.requestOutput((640, 400))
monoRightOut = monoRight.requestOutput((640, 400))
monoLeftOut.link(stereo.left)
monoRightOut.link(stereo.right)

stereo.setRectification(True)
stereo.setExtendedDisparity(True)
# stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)  # если нужно — раскомментируй

topLeft = dai.Point2f(center_x - point_half_size, center_y - point_half_size)
bottomRight = dai.Point2f(center_x + point_half_size, center_y + point_half_size)

config = dai.SpatialLocationCalculatorConfigData()
config.depthThresholds.lowerThreshold = 50
config.depthThresholds.upperThreshold = 15000
config.calculationAlgorithm = dai.SpatialLocationCalculatorAlgorithm.MEDIAN
config.roi = dai.Rect(topLeft, bottomRight)

spatialLocationCalculator.inputConfig.setWaitForMessage(False)
spatialLocationCalculator.initialConfig.addROI(config)

xoutSpatialQueue = spatialLocationCalculator.out.createOutputQueue()
outputDepthQueue = spatialLocationCalculator.passthroughDepth.createOutputQueue()

stereo.depth.link(spatialLocationCalculator.inputDepth)

inputConfigQueue = spatialLocationCalculator.inputConfig.createInputQueue()

with pipeline:
    pipeline.start()
    while pipeline.isRunning():
        spatialData = xoutSpatialQueue.get().getSpatialLocations()

        print("WASD — двигать точку, 1–5 — алгоритм, q — выход")

        outputDepthIMage : dai.ImgFrame = outputDepthQueue.get()
        frameDepth = outputDepthIMage.getFrame()        

        # Визуализация
        depthFrameColor = cv2.normalize(frameDepth, None, 255, 0, cv2.NORM_MINMAX, cv2.CV_8U)
        depthFrameColor = cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_JET)

        newConfig = False
        stepSize = 0.008

        for depthData in spatialData:
            roi = depthData.config.roi
            roi = roi.denormalize(width=depthFrameColor.shape[1], height=depthFrameColor.shape[0])

            xmin = int(roi.topLeft().x)
            ymin = int(roi.topLeft().y)
            xmax = int(roi.bottomRight().x)
            ymax = int(roi.bottomRight().y)

            cv2.rectangle(depthFrameColor, (xmin, ymin), (xmax, ymax), (0, 255, 100), 1)
            cx = (xmin + xmax) // 2
            cy = (ymin + ymax) // 2
            cv2.drawMarker(depthFrameColor, (cx, cy), (0, 255, 100), cv2.MARKER_CROSS, 18, 2)

            coords = depthData.spatialCoordinates
            txt = f"X:{int(coords.x):4d} Y:{int(coords.y):4d} Z:{int(coords.z):4d} mm"
            cv2.putText(depthFrameColor, txt, (xmin, ymin - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 1)

        cv2.imshow("depth + point", depthFrameColor)

        key = cv2.waitKey(1)

        if key == ord('q'):
            pipeline.stop()
            break

        # Движение точки
        if key == ord('w') and center_y - stepSize >= point_half_size:
            center_y -= stepSize
            newConfig = True
        elif key == ord('s') and center_y + stepSize <= 1 - point_half_size:
            center_y += stepSize
            newConfig = True
        elif key == ord('a') and center_x - stepSize >= point_half_size:
            center_x -= stepSize
            newConfig = True
        elif key == ord('d') and center_x + stepSize <= 1 - point_half_size:
            center_x += stepSize
            newConfig = True

        # Смена алгоритма
        alg_map = {
            ord('1'): dai.SpatialLocationCalculatorAlgorithm.MEAN,
            ord('2'): dai.SpatialLocationCalculatorAlgorithm.MIN,
            ord('3'): dai.SpatialLocationCalculatorAlgorithm.MAX,
            ord('4'): dai.SpatialLocationCalculatorAlgorithm.MODE,
            ord('5'): dai.SpatialLocationCalculatorAlgorithm.MEDIAN,
        }
        if key in alg_map:
            config.calculationAlgorithm = alg_map[key]
            print(f"Алгоритм: {alg_map[key].name}")
            newConfig = True

        if newConfig:
            topLeft.x = center_x - point_half_size
            topLeft.y = center_y - point_half_size
            bottomRight.x = center_x + point_half_size
            bottomRight.y = center_y + point_half_size

            config.roi = dai.Rect(topLeft, bottomRight)

            cfg = dai.SpatialLocationCalculatorConfig()
            cfg.addROI(config)
            inputConfigQueue.send(cfg)

cv2.destroyAllWindows()