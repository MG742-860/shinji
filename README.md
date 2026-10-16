# shinji
Global Localization



### Build from source

```
# build teaser++ library
cd thirdparty/teaserpp
mkdir build && cd build
cmake .. && make -j 12

# build shinji ros package
catkin build shinji -DCMAKE_BUILD_TYPE=Release
```



### Pointcloud map preprocess

```
cd thirdparty/preprocess
mkdir build && cd build
cmake .. && make -j 12

./preprocess ../config.yaml
```



### usage

```
rosservice call /shinji/query
```

```
rostopic echo /shinji/result
```



### Scan Context yaw prior (for symmetric environments)

In highly symmetric maps the global matcher may lock onto the mirrored/opposite
pose. UWB supplies the *translation* of the initial guess, and Scan Context
supplies the missing *yaw* so the matcher starts from the correct orientation.

On every `/shinji/query` request with `use_guess=true`:

1. the UWB position (`guess_pose.position`) is used as the crop center,
2. the global map is cropped within `crop_radius` around that position and
   re-centered on the robot,
3. a Scan Context descriptor is built for the cropped map and for the recently
   aggregated query clouds,
4. `distanceBtnScanContext` aligns the two descriptors and the column shift is
   converted into a yaw, which is written into the rotation of the guess,
5. the resulting `{translation from UWB, rotation from scancontext}` guess is
   passed to `shinji::try_request`.

Enable it in the config file:

```json
"scancontext": {
  "enable": true,
  "map_type": "filtered",      // "origin" or "filtered"
  "crop_radius": 80.0,         // radius (m) cropped around the UWB position
  "voxel_resolution": 0.2,     // downsampling (m) before building the descriptor
  "dist_thres": 0.5,           // reject the result when the descriptor distance is larger
  "min_points": 100,           // minimum points required in the cropped map
  "yaw_sign": 1.0,             // flip to -1 if the yaw convention comes out inverted
  "lidar_height": 0.0          // lidar height relative to the UWB antenna
}
```

The query buffer length reuses `gicp.source_frames`, so no extra parameter is
needed. `map_type` selects which globalmap PCD is cropped (`common.globalmap_filtered`
or `common.globalmap_origin`).


