import depthai as dai
import cv2

pipeline = dai.Pipeline()
cam = pipeline.createColorCamera()
cam.setPreviewSize(640, 480)
cam.setInterleaved(False)
xout = pipeline.createXLinkOut()
xout.setStreamName("rgb")
cam.preview.link(xout.input)

with dai.Device(pipeline) as device:
    q = device.getOutputQueue("rgb", maxSize=4, blocking=False)
    while True:
        frame = q.get().getCvFrame()
        cv2.imshow("OAK-D Preview", frame)
        if cv2.waitKey(1) == ord('q'):
            break
cv2.destroyAllWindows()
