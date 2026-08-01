# Target measurement timestamp interface

The formal Simulink loop consumes one `std_msgs/msg/Float64MultiArray` on
`/apriltag_detector/target_position` with exactly eight elements:

```text
[valid,
 velocity_valid,
 X_C,
 Y_C,
 v_target_x_C,
 v_target_y_C,
 stamp_sec,
 stamp_nanosec]
```

The first six fields retain their existing meanings. Internally,
`v_target_x_C` and `v_target_y_C` are target-to-camera relative velocity.

`stamp_sec` and `stamp_nanosec` must be copied from the Header stamp of the
camera image used to produce the four position/velocity values. They must not
be generated at publish completion. The integer seconds and nanoseconds are
transported separately to avoid loss of timestamp precision.

The message is invalid for closed-loop use when the image stamp is zero,
non-finite, non-integral, negative, or has `stamp_nanosec >= 1e9`. In that
case both validity flags must be zero. Legacy six-element messages are rejected
by the formal model; `allow_legacy_untimestamped=false`.

The repository's current `apriltag_detector.py` publishes the older
three-element pixel interface `[valid,u,v]` and does not implement the
`X_C/Y_C/relative velocity` estimator. Therefore it is not modified to mimic
this interface. The actual six-element motion-estimation publisher must append
the source image Header fields described above.
