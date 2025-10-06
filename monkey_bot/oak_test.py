import depthai as dai
import cv2

# Build pipeline
pipeline = dai.Pipeline()

cam = pipeline.create(dai.node.ColorCamera)
cam.setPreviewSize(640, 480)
cam.setInterleaved(False)
cam.setFps(30)

xout = pipeline.create(dai.node.XLinkOut)
xout.setStreamName("rgb")
cam.preview.link(xout.input)

# Run
with dai.Device(pipeline) as dev:
    q = dev.getOutputQueue("rgb", maxSize=4, blocking=False)
    while True:
        frame = q.get().getCvFrame()
        cv2.imshow("OAK-D Preview", frame)
        if cv2.waitKey(1) == ord('q'):
            break
cv2.destroyAllWindows()
