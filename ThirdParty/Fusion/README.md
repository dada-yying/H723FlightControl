# Pinned x-io Fusion AHRS

Source: https://github.com/xioTechnologies/Fusion

Commit: `a8d7224f36a0ec82345ef49a3db50e65f8d3bab8`, inspected 2026-10-03.
License: MIT, retained verbatim in `LICENSE.md`.

Vendored subset: FusionAhrs.c/.h, FusionMath.h, FusionConfig.h,
FusionInline.h, FusionConvention.h. Only FusionAhrs.c needs compilation;
the unused convention-to-string API is not linked or called.

The only upstream modification is enabling `FUSION_USE_NORMAL_SQRT` in
FusionConfig.h. H723 has an FPU; this avoids fast-inverse-square-root error
and type-punning concerns and keeps host/ARM normalization comparable.

The local adapter is App/bmi088_attitude.c. It supplies calibrated sensor
values, proper FRD axis alignment, gyro deg/s, accel g, measured sample dt,
nonblocking initial gravity alignment, finite/range/freshness checks and
paired raw/attitude publication. No FusionBias, magnetometer, external heading
or control-loop integration is enabled. Never silently replace this pinned
source with a new upstream API without rerunning host and ARMCC tests.
