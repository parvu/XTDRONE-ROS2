# 47. Sensor models

!!! success "Rewritten for Gazebo Harmonic"
    Background chapter. Sensors are SDF `<sensor>` elements simulated by gz-sensors, instead of Classic plugins.

## IMU, magnetometer, barometer, GPS

Gazebo Harmonic simulates them with the world systems `Imu`, `Magnetometer`, `AirPressure` and `NavSat`. The sensors sit
on `base_link` of each vehicle (`imu_sensor`, `magnetometer_sensor`, `air_pressure_sensor`, `navsat_sensor`). PX4's
`gz_bridge` reads them. The noise model per axis is

\[
x_m = x + b + n,\qquad \dot b = n_b
\]

with white noise \(n\) and a random-walk bias driven by \(n_b\). In SDF:

```xml
<sensor name="imu_sensor" type="imu">
  <update_rate>250</update_rate>
  <imu>
    <angular_velocity>
      <x><noise type="gaussian">
        <stddev>0.0003394</stddev>                          <!-- gyroscopeNoiseDensity -->
        <dynamic_bias_stddev>3.8785e-05</dynamic_bias_stddev>   <!-- gyroscopeRandomWalk -->
        <dynamic_bias_correlation_time>1000</dynamic_bias_correlation_time>
      </noise></x>
      ...
    </angular_velocity>
    <linear_acceleration> ... stddev 0.004, dynamic_bias_stddev 0.006, correlation time 300 ... </linear_acceleration>
  </imu>
</sensor>
```

| Classic `libgazebo_imu_plugin` | gz SDF noise element |
|---|---|
| `NoiseDensity` | `<stddev>` |
| `RandomWalk` | `<dynamic_bias_stddev>` |
| `BiasCorrelationTime` | `<dynamic_bias_correlation_time>` |
| `TurnOnBiasSigma` | `<bias_stddev>` |

GPS noise used to be compiled into `gazebo_gps_plugin.h`. With `navsat_sensor` it is set in SDF (`<position_sensing>`,
`<velocity_sensing>` with horizontal/vertical noise). The magnetometer's noise is set in tesla, while the sensor
reports gauss (see the comment in PX4's `x500_base`).

## Stereo camera

Gazebo Harmonic has no `multicamera` sensor. The XTDrone stereo kit uses two `camera` sensors on one link, 0.12 m apart:

| Parameter | Element | XTDrone value |
|---|---|---|
| image size, format | `<image><width>`, `<height>`, `<format>` | 752 × 480, R8G8B8 |
| field of view | `<horizontal_fov>` | 1.5708 rad |
| clipping | `<clip><near>`, `<far>` | 0.1 m, 300 m |
| baseline | `<pose>` of the two sensors | ±0.06 m in y |
| rate | `<update_rate>` | 30 Hz |
| noise | `<noise>` | Gaussian, σ = 0.007 |

Gazebo computes the intrinsics from the image size and FOV: \(f_x=f_y=\frac{w/2}{\tan(\text{fov}/2)}\) = 376 px,
\(c_x\)=376, \(c_y\)=240. These are the values of the Classic plugin. Distortion can be added with
`<distortion><k1>…</k1></distortion>`. Read the published intrinsics with
`ros2 topic echo --once /iris_0/stereo_camera/left/camera_info`.

## Depth camera

The RealSense kit is an `rgbd_camera` sensor: colour image, depth image and point cloud from one sensor.
Size 640 × 480, FOV 1.047 rad, 20 Hz, depth range 0.1–10 m. Topics are listed in chapter 12.

## Lidars

Both lidars are `gpu_lidar` sensors, which are rendered and therefore also see actors:

| | 2D (`iris_2d_lidar`) | 3D (`iris_3d_lidar`) |
|---|---|---|
| horizontal samples / range | 512 / ±π | 512 / ±π |
| vertical samples / range | 1 | 32 / ±15° |
| distance | 0.1–20 m | 0.3–30 m |
| rate | 40 Hz | 15 Hz |
| noise | Gaussian σ = 0.01 m | Gaussian σ = 0.01 m |
