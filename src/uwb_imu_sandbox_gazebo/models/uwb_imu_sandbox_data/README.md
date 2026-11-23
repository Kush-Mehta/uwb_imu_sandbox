# Sealed sensor models

`uwb_model.bin` and `imu_model.bin` hold the fitted sensor tables used by the
`UwbDevice` (tag) and `RealisticImu` plugins. They were built from real
measurements by a private tool. The files do not contain the measurements,
and the UWB signal fields in them are already on a unitless 0..100 scale.

Do not edit them; the plugins check their integrity on load.
