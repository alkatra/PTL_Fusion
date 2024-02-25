// Attach click event listener to the "startCapture" button to initiate video capture
document.getElementById("startCapture").addEventListener("click", async () => {
  // Define constraints for video and audio capture, targeting the rear-facing camera with specific resolutions and frame rates
  const constraints = {
    video: {
      facingMode: "environment",
      width: { ideal: 1080, min: 1080 },
      height: { ideal: 1920, min: 1920 },
      frameRate: { ideal: 30, min: 30 },
    },
    audio: true,
  };

  // Request access to the media devices with specified constraints
  const stream = await navigator.mediaDevices.getUserMedia(constraints);

  // Options for the media recorder, including MIME type and video bitrate
  const mediaRecorderOptions = {
    mimeType: "video/webm;",
    videoBitsPerSecond: 20 * 1024 * 1024,
  };

  // Initialize the media recorder with the stream and options
  const mediaRecorder = new MediaRecorder(stream, mediaRecorderOptions);

  // Setup to store recorded data chunks and a counter for video numbering
  let chunks = [];
  let videoNumber = 1;

  // Collect data chunks when available
  mediaRecorder.ondataavailable = (event) => {
    if (event.data.size > 0) {
      chunks.push(event.data);
    }
  };

  // Process and send data chunks as a single video file when recording stops
  mediaRecorder.onstop = async () => {
    const blob = new Blob(chunks, { type: "video/webm" }); // Combine chunks into a blob
    chunks = []; // Reset chunks for next recording

    // Prepare the video file to be sent to the server
    const formData = new FormData();
    const uniqueFilename = `video_${videoNumber++}_${Date.now()}.webm`;
    formData.append("file", blob, uniqueFilename);

    // Send the video file to the server
    const response = await fetch("/upload_video/", {
      method: "POST",
      body: formData,
    });

    const responseData = await response.text(); // Log server response
    console.log(responseData);
  };

  // Setup recording duration and gap between recordings
  const captureDuration = 250; // Duration of each recording in milliseconds
  const gapDuration = 0; // Gap between recordings, set to continuous recording

  // Function to start recording if not already in progress
  const startRecording = () => {
    if (mediaRecorder.state !== "recording") {
      mediaRecorder.start(); // Start recording
      setTimeout(() => {
        if (mediaRecorder.state === "recording") {
          mediaRecorder.stop(); // Stop recording after specified duration
        }
      }, captureDuration);
    }
  };

  // Start the first recording immediately
  startRecording();

  // Continuously start new recordings according to the specified duration and gap
  setInterval(() => {
    startRecording();
  }, captureDuration + gapDuration);
});
