import numpy as np
from faster_whisper import WhisperModel
import wave
import sys

# ── load WAV ──────────────────────────────────────────────────────────────────
def load_wav(path: str) -> np.ndarray:
    with wave.open(path, 'rb') as wf:
        assert wf.getnchannels() == 1,   'WAV must be mono'
        assert wf.getsampwidth() == 2,   'WAV must be 16-bit'
        assert wf.getframerate() == 16000, 'WAV must be 16kHz'
        raw = wf.readframes(wf.getnframes())
    return np.frombuffer(raw, dtype=np.int16).astype(np.float32) / 32768.0

# ── intent map (same as STT node) ────────────────────────────────────────────
INTENT_MAP = {
    'follow': [
        'come here', 'come to me', 'follow me', 'follow',
        'come', 'get here', 'come over', 'approach me',
    ],
}

def classify_intent(text: str) -> str:
    lowered = text.lower().strip()
    for intent, phrases in INTENT_MAP.items():
        for phrase in phrases:
            if phrase in lowered:
                return intent
    return 'unknown'

def main():
    # move the existing top-level code into here
    import sys
    # wav_path = sys.argv[1] if len(sys.argv) > 1 else 'test.wav'
    wav_path = "src/frank_audio/resource/test.wav"
    print(f'Loading Whisper tiny.en...')
    model = WhisperModel('tiny.en', device='cpu', compute_type='int8')

    print(f'Loading WAV: {wav_path}')
    audio = load_wav(wav_path)
    print(f'Duration: {len(audio)/16000:.2f}s')

    print('Transcribing...')
    segments, info = model.transcribe(audio, language='en', beam_size=1, vad_filter=True)
    text = ' '.join(seg.text.strip() for seg in segments).strip()

    print(f'\nTranscript : "{text}"')
    print(f'Intent     : {classify_intent(text)}')

if __name__ == '__main__':
    main()