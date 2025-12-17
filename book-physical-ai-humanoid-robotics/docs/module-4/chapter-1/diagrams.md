# Voice Processing Pipeline Diagrams

## Diagram 1: Basic Voice-to-Action Pipeline

```
[User Speaks] -> [Audio Capture] -> [Whisper ASR] -> [NLP Processing] -> [Intent Mapping] -> [Robot Action] -> [Execution Feedback]
```

Description: Shows the complete flow from voice input to robot action execution, highlighting the key processing steps.

## Diagram 2: Whisper Integration in Robotics

```
[Robot Microphones] -> [Audio Preprocessing] -> [Whisper Model] -> [Text Output]
                                              |
                                              v
                                    [Language Understanding] -> [Action Planning]
```

Description: Illustrates how Whisper fits into the robotic system architecture, showing the audio input path and text processing path.

## Diagram 3: Latency Analysis

```
Audio Capture (50-100ms) -> Whisper Processing (200-500ms) -> NLP (100-200ms) -> Action Execution (variable)
```

Description: Breaks down the latency contributions of each step in the voice processing pipeline.

## Diagram 4: Intent Mapping Process

```
Raw Text -> [Intent Classifier] -> [Parameter Extractor] -> [Action Generator] -> [Robot Command]
```

Description: Shows the step-by-step process of converting raw text to executable robot commands.