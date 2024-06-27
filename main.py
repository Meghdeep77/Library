from googleapiclient.discovery import build
import re
from datetime import datetime, timedelta
from moviepy.editor import VideoFileClip,AudioFileClip
from pytube import YouTube
import cv2
# Replace 'YOUR_API_KEY' with your actual API key
api_key = 'AIzaSyDoVkEyPUdD6OUW-Dr_2pfbJhgU7hUtG-s'
# Replace 'VIDEO_ID' with the actual video ID from which you want to retrieve comments
video_url = 'https://www.youtube.com/watch?v=_32QyCVw6Ig'

db = {}
# Build the service object
youtube = build('youtube', 'v3', developerKey=api_key)

# Function to get comments
def extract_video_id_from_url(url):
    if 'youtu.be/' in url:
        video_id = url.split('/')[-1]
    else:
        video_id = url.split('v=')[1].split('&')[0]

    return video_id

def download_youtube_video(video_url, download_path):
    try:
        # Create a YouTube object
        yt = YouTube(video_url)

        # Get the highest resolution stream
        stream = yt.streams.get_highest_resolution()

        # Download the video
        video_file_path = stream.download(output_path=download_path)

        print(f"Downloaded: {yt.title}")

        return video_file_path
    except Exception as e:
        print(f"An error occurred: {e}")
        return None
def get_comments(youtube, video_id):
    # Call the commentThreads.list method to retrieve comments
    request = youtube.commentThreads().list(
        part='snippet',
        videoId=video_id,
        maxResults=100,  # Change this value to get more or fewer comments
        textFormat='plainText'
    )
    response = request.execute()

    comments = []
    for item in response['items']:
        comment = item['snippet']['topLevelComment']['snippet']['textDisplay']
        comments.append(comment)

    return comments

def contains_timestamp(comment):
    # Regular expression to match typical timestamp patterns like "1:23", "10:45", etc.
    timestamp_pattern = re.compile(r'\b\d{1,2}:\d{2}\b')
    return bool(timestamp_pattern.search(comment))

def extract_timestamps(comment):
    # Regular expression to match typical timestamp patterns like "1:23", "10:45", "01:23:45", etc.
    timestamp_pattern = re.compile(r'\b\d{1,2}:\d{2}(?::\d{2})?\b')
    timestamps = timestamp_pattern.findall(comment)
    return timestamps
def add_seconds_to_timestamp(timestamp, seconds):
    if len(timestamp.split(':')) == 2:
        # Format: MM:SS
        time_format = '%M:%S'
    else:
        # Format: HH:MM:SS
        time_format = '%H:%M:%S'

    # Parse the timestamp into a datetime object
    time_obj = datetime.strptime(timestamp, time_format)
    # Add the seconds
    new_time_obj = time_obj + timedelta(seconds=seconds)

    # Format the new datetime object back to a string
    if len(timestamp.split(':')) == 2:
        return new_time_obj.strftime('%M:%S')
    else:
        return new_time_obj.strftime('%H:%M:%S')


def time_to_seconds(time_str):
    # Regular expression to match time in the format HH:MM:SS or MM:SS
    time_pattern = re.compile(r'(\d+):(\d+)(?::(\d+))?')
    match = time_pattern.match(time_str)

    if match:
        hours, minutes, seconds = match.groups()
        if seconds is None:
            seconds = 0
        return int(hours or 0) * 3600 + int(minutes) * 60 + int(seconds)
    else:
        raise ValueError(f"Invalid time format: {time_str}")
# Fetch and print comments

def extract_clips(video_path, clips):
    # Load the video
    video = VideoFileClip(video_path)

    for idx, (clip_name, times) in enumerate(clips.items()):
        start_time = times['start']
        end_time = times['end']

        # Extract the subclip
        clip = video.subclip(start_time, end_time)

        target_width = 720  # Example width for shorts format
        target_height = 1280  # Example height for shorts format

        # Resize the video while preserving aspect ratio
          # Use BILINEAR method

        # Define the output file path
        output_file = f'clip_{idx + 1}.mp4'

        # Write the resized subclip to a file
        clip.write_videofile(output_file, codec='libx264')


def resize_and_add_audio(input_video_path, output_video_path, target_width, target_height):
    # Extract the audio from the input video
    def extract_audio_from_video(source_video_path, audio_output_path):
        video = VideoFileClip(source_video_path)
        audio = video.audio
        audio.write_audiofile(audio_output_path)
        video.close()

    # Resize the video
    def resize_video(input_video_path, output_video_path, target_width, target_height):
        cap = cv2.VideoCapture(input_video_path)

        if not cap.isOpened():
            print(f"Error: Video file not found at {input_video_path}")
            return False

        frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(f"Original video dimensions: {frame_width}x{frame_height}")

        if frame_width <= 0 or frame_height <= 0 or target_width <= 0 or target_height <= 0:
            print("Error: Video frame dimensions and target dimensions must be positive")
            return False

        new_width = int(target_width)
        new_height = int(target_height)
        print(f"New video dimensions: {new_width}x{new_height}")

        out = cv2.VideoWriter(output_video_path, cv2.VideoWriter_fourcc(*'mp4v'), cap.get(cv2.CAP_PROP_FPS),
                              (new_width, new_height))

        while True:
            ret, frame = cap.read()
            if not ret:
                break
            resized_frame = cv2.resize(frame, (new_width, new_height))
            out.write(resized_frame)

        cap.release()
        out.release()
        return True

    # Add the extracted audio to the resized video
    def add_audio_to_video(target_video_path, audio_input_path, output_video_path):
        video = VideoFileClip(target_video_path)
        audio = AudioFileClip(audio_input_path)
        video = video.set_audio(audio)
        video.write_videofile(output_video_path, codec='libx264', audio_codec='aac')
        video.close()
        audio.close()

    audio_path = "extracted_audio.mp3"
    resized_video_path = "resized_video.mp4"

    # Extract audio
    extract_audio_from_video(input_video_path, audio_path)

    # Resize video
    if resize_video(input_video_path, resized_video_path, target_width, target_height):
        # Add audio to resized video
        add_audio_to_video(resized_video_path, audio_path, output_video_path)
        print("Video resizing and audio processing completed successfully.")
    else:
        print("Failed to resize the video.")


video_id = extract_video_id_from_url(video_url)
comments = get_comments(youtube, video_id)
clip_no = 0
for idx, comment in enumerate(comments):
    timestamps = extract_timestamps(comment)
    if timestamps:
        print(f"Comment {idx + 1}: {comment}")
        clip_no = clip_no + 1
        for timestamp in timestamps:
            print(f"  Extracted timestamp: {timestamp}")
            print(f"  End Time stamp: {add_seconds_to_timestamp(timestamp,15)}")
            db[f"Clip {clip_no}"] = {"start": timestamp, "end": add_seconds_to_timestamp(timestamp,15)}
    else:
        continue

print(db)

video_path = download_youtube_video(video_url,r"C:\Users\Meghdeep\PycharmProjects\smartvidclipper")
print(video_path)
extract_clips(video_path, db)
for i in range(len(db)):
    resize_and_add_audio(f'clip_{i+1}.mp4',f'clip{i+1}_final.mp4',360,450)


