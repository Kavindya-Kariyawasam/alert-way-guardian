import sys
import asyncio
import logging
from pathlib import Path
import websockets
from pydub import AudioSegment
import io
import signal
import time 
from azure.storage.blob import BlobServiceClient, BlobClient, ContainerClient
from pydub import AudioSegment, effects
from pydub.effects import normalize, compress_dynamic_range, high_pass_filter, low_pass_filter
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

class AudioServer:
    def __init__(self, connection_string, save_path='output'):
        self.save_path = Path(save_path)
        self.save_path.mkdir(parents=True, exist_ok=True)
        self.final_audio_path = Path('finalAudio')
        self.final_audio_path.mkdir(parents=True, exist_ok=True)
        self.audio_chunks = []
        self.clients = set()
        self.recording = False
        self.file_counter = 0
        self.saving_audio = False
        self.init_azure_storage(connection_string)

    def init_azure_storage(self, connection_string):
        self.blob_service_client = BlobServiceClient.from_connection_string(connection_string)
        self.container_client = self.blob_service_client.get_container_client(
            os.getenv('AZURE_CONTAINER_NAME')
        )

    async def register(self, websocket):
        self.clients.add(websocket)
        logging.info(f"New client connected. Total clients: {len(self.clients)}")

    async def unregister(self, websocket):
        self.clients.remove(websocket)
        logging.info(f"Client disconnected. Total clients: {len(self.clients)}")

    async def consumer_handler(self, websocket):
        try:
            async for message in websocket:
                if message == 'start_recording':
                    while self.saving_audio:
                        await asyncio.sleep(0.1)
                    self.recording = True
                    self.audio_chunks = []
                    logging.info("Recording started")
                elif message == 'stop_recording':
                    self.recording = False
                    logging.info("Recording stopped")
                    asyncio.create_task(self.save_audio())
                elif message == 'emergency_deactivated':
                    logging.info("Emergency deactivated. Merging all MP3 files.")
                    await self.merge_mp3_files()
                elif self.recording:
                    self.audio_chunks.append(message)
        except websockets.exceptions.ConnectionClosedError:
            logging.info("Connection closed")
        finally:
            await self.unregister(websocket)

    async def handler(self, websocket):
        await self.register(websocket)
        await self.consumer_handler(websocket)

    async def save_audio(self):
        self.saving_audio = True
        try:
            if not self.audio_chunks:
                logging.warning("No audio data to save")
                return

            timestamp = time.strftime("%Y%m%d-%H%M%S")
            file_prefix = f"{timestamp}_{self.file_counter:03d}"
            pcm_filename = f'{file_prefix}_output.pcm'
            mp3_filename = f'{file_prefix}_output.mp3'

            # Save locally
            pcm_path = self.save_path / pcm_filename
            mp3_path = self.save_path / mp3_filename

            logging.info(f"Saving PCM audio locally to {pcm_path}")
            with open(pcm_path, 'wb') as f:
                for chunk in self.audio_chunks:
                    f.write(chunk)

            logging.info(f"Converting to MP3 and saving locally to {mp3_path}")
            audio = AudioSegment.from_raw(io.BytesIO(b''.join(self.audio_chunks)), 
                                          sample_width=2, 
                                          frame_rate=44100, 
                                          channels=1)
            
            audio.export(mp3_path, format="mp3")

            # Save to Azure Blob Storage
            logging.info(f"Saving PCM audio to Azure Blob Storage: {pcm_filename}")
            pcm_blob_client = self.container_client.get_blob_client(pcm_filename)
            pcm_data = b''.join(self.audio_chunks)
            pcm_blob_client.upload_blob(pcm_data, overwrite=True)

            logging.info(f"Saving MP3 to Azure Blob Storage: {mp3_filename}")
            mp3_blob_client = self.container_client.get_blob_client(mp3_filename)
            with open(mp3_path, "rb") as mp3_file:
                mp3_blob_client.upload_blob(mp3_file, overwrite=True)

            logging.info("Audio saved successfully both locally and to Azure Blob Storage")
            self.file_counter += 1
        finally:
            self.saving_audio = False
            self.audio_chunks = []

    async def merge_mp3_files(self):
        mp3_files = list(self.save_path.glob('*.mp3'))
        if not mp3_files:
            logging.warning("No MP3 files found to merge.")
            return

        merged_audio = AudioSegment.empty()
        for mp3_file in sorted(mp3_files):
            audio = AudioSegment.from_mp3(mp3_file)
            merged_audio += audio

        timestamp = time.strftime("%Y%m%d-%H%M%S")
        final_audio_filename = f'final_audio_{timestamp}.mp3'
        final_audio_path = self.final_audio_path / final_audio_filename

        boost_amount = 35  # Total desired boost in dB
        increment = 5  # Boost by 5 dB at a time
        boosted_audio = merged_audio
        
        for _ in range(0, boost_amount, increment):
            boosted_audio += increment
            # Apply a limiter after each boost
            compressed_audio = effects.compress_dynamic_range(boosted_audio,
                threshold=-1,
                ratio=20,
                attack=5,
                release=50
            )

        # Save merged audio locally
        compressed_audio.export(final_audio_path, format="mp3")
        logging.info(f"Merged audio saved locally to {final_audio_path}")

        # Save merged audio to Azure Blob Storage
        final_audio_blob_client = self.container_client.get_blob_client(final_audio_filename)
        with open(final_audio_path, "rb") as final_audio_file:
            final_audio_blob_client.upload_blob(final_audio_file, overwrite=True)
        logging.info(f"Merged audio saved to Azure Blob Storage: {final_audio_filename}")

async def main():
    host = os.getenv('SERVER_HOST', '0.0.0.0')
    port = int(os.getenv('SERVER_PORT', '8080'))
    azure_connection_string = os.getenv('AZURE_STORAGE_CONNECTION_STRING')
    
    if not azure_connection_string:
        raise ValueError("Azure Storage connection string not found in environment variables")
    
    audio_server = AudioServer(azure_connection_string)
    stop = asyncio.Event()
    
    def signal_handler(signum, frame):
        logging.info("Shutting down...")
        stop.set()

    if sys.platform != 'win32':
        loop = asyncio.get_running_loop()
        for sig in (signal.SIGINT, signal.SIGTERM):
            loop.add_signal_handler(sig, lambda s=sig: signal_handler(s, None))
    else:
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)

    async with websockets.serve(audio_server.handler, host, port):
        logging.info(f"Server listening on {host}:{port}")
        try:
            await stop.wait()
        except asyncio.CancelledError:
            pass

if __name__ == '__main__':
    asyncio.run(main())
