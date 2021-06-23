# visualize_feature_tracks

The goal of this package is to test visual feature tracking on an input image stream.

## run:

There is only one launch file that needs to be run, and all params are set in this launch file:

```
roslaunch visualize_feature_tracks track_features.launch
```

### Notes on setting launch file parameters:

 * For variable type options, if you enter an invalid arugment, it will list the options for you
 * A descriptor is not needed for KLT tracker, so you can set it to NONE if you are using KLT and it should speed up processing.
 * Entering an empty string for the config paths will use default variables defined in the classes
