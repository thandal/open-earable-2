# Gesture Detection Pipeline

This note summarizes the gesture-detection pipeline as it is described across the LaTeX sources in:

- `tex/chapter/functionality.tex`
- `tex/chapter/analysis.tex`
- `tex/chapter/design.tex`

The gesture-control goal is to detect tap interactions on the OpenEarable and map them to media commands such as play/pause and track navigation.

## High-level idea

The LaTeX chapters describe a modular signal-processing pipeline built as a directed graph of stages. For gesture detection, the main sensing source is the IMU. A tap is modeled as a short event that produces:

- a brief acceleration burst
- a small device tilt or rotation

The current concrete detection path described in the analysis chapter is a rule-based tap detector that primarily uses the accelerometer, especially the `z` axis.

## Configured pipeline

The configured accelerometer-based pipeline is:

```text
Accelerometer (x, y, z)
        |
        v
Select z-axis
        |
        v
4th-order Butterworth bandpass
(0.5 Hz to 10 Hz)
        |
        v
Absolute magnitude
        |
        v
Sliding-window local peak search
        |
        v
Amplitude thresholding
(theta_low <= peak <= theta_high)
        |
        v
Cooldown check
        |
        v
Tap event prediction
        |
        v
Tap count / command mapping
```

## Stage-by-stage explanation

### 1. IMU input

The overall gesture idea uses the IMU because the device has no dedicated touch sensor. A tap on the earable creates a short acceleration response and usually a small rotational motion.

### 2. Accelerometer focus

Although the accelerometer provides all three axes, the LaTeX analysis states that the `z` axis shows the clearest tap peaks and is therefore used as the main classification input.

### 3. Bandpass filtering

The selected `z`-axis signal is processed by a fourth-order Butterworth bandpass filter with:

- low cutoff: `0.5 Hz`
- high cutoff: `10 Hz`

This is intended to suppress slow drift and high-frequency noise while preserving the characteristic tap pattern.

### 4. Magnitude conversion

After filtering, the signal is converted to an absolute magnitude:

```text
s(t) = |filtered_z(t)|
```

This makes positive and negative tap excursions contribute equally to detection.

### 5. Sliding-window peak detection

The filtered magnitude is scanned for local maxima. The thesis text describes this as a window-based search for short-duration peaks that match the structure of a tap event.

### 6. Threshold check

Only local peaks inside a predefined amplitude range are considered valid tap candidates:

```text
theta_low <= peak <= theta_high
```

This removes weak peaks and overly large motion artifacts.

### 7. Cooldown

A cooldown `C` is applied after each accepted peak so that one physical tap is not counted multiple times.

### 8. Event expansion

When a valid peak is found, the detector marks a small neighborhood around the peak as the predicted tap region. In the LaTeX description this is written as a prediction window around the detected peak index.

## Why this configuration was chosen

According to the analysis chapter, tap events are visible in raw accelerometer data, but become much easier to separate from normal movement after bandpass filtering. The filtered `z`-axis signal shows clearer tap peaks with more consistent duration and energy, which makes a lightweight rule-based detector feasible.

This fits the overall implementation approach from the design chapter:

- use modular processing stages
- keep the pipeline lightweight enough for embedded execution
- mirror the same graph in Python for fast iteration before firmware deployment

## Gyroscope role

The LaTeX analysis also discusses gyroscope data. It suggests that the gyroscope captures related rotational motion during taps, but it does not describe a fully specified standalone gyroscope classifier. Instead, it points toward a possible late-fusion setup where gyroscope information could support the accelerometer-based decision.

So based on the current LaTeX files:

- the clearly specified detector is the accelerometer-based filter pipeline
- gyroscope fusion is discussed, but not fully configured in detail yet

## Output to gesture control

Once tap events are detected, they are intended to drive gesture-control actions such as:

- single tap -> start/stop
- double tap -> next track
- triple tap -> previous track

The functionality chapter presents this as the command mapping layer on top of the sensor pipeline.

## Short summary

The gesture-detection pipeline described in the LaTeX files is a modular IMU-based tap detector with a concrete rule-based core:

1. read IMU data
2. use the accelerometer `z` axis
3. bandpass filter it from `0.5 Hz` to `10 Hz`
4. take the absolute magnitude
5. detect local peaks in a sliding window
6. keep only peaks inside a valid amplitude range
7. suppress duplicate detections with a cooldown
8. convert accepted peaks into tap events for media-control mapping
