from fastapi import FastAPI, File, UploadFile, Form
from fastapi.responses import PlainTextResponse, JSONResponse, HTMLResponse, RedirectResponse
import aiofiles
import os
from fastapi.staticfiles import StaticFiles
from fastapi.middleware.cors import CORSMiddleware
from typing import List
from video_processor import VideoProcessor
import uvicorn

# Initialize FastAPI app
app = FastAPI()

# Configure CORS middleware for cross-origin requests handling
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Allow all origins
    allow_credentials=True,
    allow_methods=["*"],  # Allow all methods
    allow_headers=["*"],  # Allow all headers
)

# Serve static files from the 'static' directory
app.mount("/static", StaticFiles(directory="static"), name="static")

# Directory for storing uploaded files. Ensure directory exists.
UPLOAD_DIRECTORY = "uploaded_files"
if not os.path.exists(UPLOAD_DIRECTORY):
    os.makedirs(UPLOAD_DIRECTORY)

# Test endpoint to check if the server is running
@app.get("/test", response_class=PlainTextResponse)
async def test_endpoint():
    return "SUCCESS"

# Endpoint to return phone.html and phone.js
@app.get("/feed", response_class=HTMLResponse)
async def read_feed():
    async with aiofiles.open('static/phone.html', mode='r') as f:
        html_content = await f.read()
    return HTMLResponse(content=html_content)

# Endpoint for video upload and processing
@app.post("/upload_video/")
async def upload_video(file: UploadFile = File(...)):
    try:
        # Save uploaded file to specified directory
        file_location = f"{UPLOAD_DIRECTORY}/{file.filename}"
        with open(file_location, "wb+") as file_object:
            file_object.write(file.file.read())
        # Process the video using a custom VideoProcessor class
        processor = VideoProcessor(video_path=file_location)
        processor.process_video()
        return {"filename": file.filename}
    except Exception as e:
        # Handle exceptions by returning an error response
        return JSONResponse(status_code=400, content={"message": f"Could not upload the video: {e}"})

# Main entry point for running the app with uvicorn
if __name__ == "__main__":
    uvicorn.run("server:app", host="192.168.12.16", port=8000, reload=True, ssl_keyfile="./key.pem", ssl_certfile="./cert.pem")
