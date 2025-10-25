import cv2

# Replace 5000 with the UDP port you used in Jetson pipeline
gstreamer_pipeline = (
    'udpsrc port=5000 caps="application/x-rtp, media=video, encoding-name=H264, payload=96" ! '
    'rtph264depay ! avdec_h264 ! videoconvert ! appsink'
)

# Open video capture from GStreamer pipeline
cap = cv2.VideoCapture(gstreamer_pipeline, cv2.CAP_GSTREAMER)

if not cap.isOpened():
    print("Failed to open video capture")
    exit(1)

while True:
    ret, frame = cap.read()
    if not ret:
        continue

    # Display the frame
    cv2.imshow("Jetson Stream", frame)

    # Press 'q' to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
