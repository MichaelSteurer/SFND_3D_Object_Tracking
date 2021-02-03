# FP 1


# FP 2


# FP 3


# FP 4


# FP 5

In the following list we show the time to collision for all the images. Overall the results are pretty stable but there are two outliers. In the third frame the TTC goes up from _11.26s_ to _19.60s_ and two frames later it goes down to _11.89s_ again. A visual inspection clearly shows that the distance between the two cars becomes smaller and smaller.

```
13.0311s, 11.2558s, 19.5975s, 17.3200s, 11.8908s, 10.0697s, 14.6173s, 14.1434s, 14.0434s, 10.5329s, 11.2431s, 8.39768s, 11.3635s, 8.12875s, 8.96666s, 11.1807s, 7.68874s
```

There is no obvious reason why the time changed so quickly but an educated guess would be the computation of the time to collision.

In the implementation of the _computeTTCLidar(...)_ we used a very simple mechanism to remove outliers. We computed the mean and standard deviation of all the distances between the two cars. For the computation of the minimum distance between the two cars we considered only points that are within the bounds of _x +- 2*stdev_. 

Due to these computations, the mean does probably not move far enough towards the ego car and so the ttc is not computed correctly. For the remaining frames we did not observe this again.


# FP 6

## Different Detectors

From the mid term project we know that _FAST_ and _BRISK_ detectors have by far the highest number of matches. So computing the time to collision using these detectors show stable results. There are no significant outliers for both but _FAST_ performs in general better. The times vary between _10_ and _14_ seconds. 

Result set for _FAST_:

```
11.80s, 11.38s, 13.08s, 12.48s, 14.97s, 13.72s, 13.07s, 11.9s, 13.41s, 11.81s, 11.33s, 12.13s, 10.38s, 10.64s, 10.81s, 10.62s, 11.02s
```

From the midterm project we also know that _ORB_ and _SIFT_ detectors are returning a relatively low number of matches. Computing the time to collision with these detectors shows that the collision times vary between _8_ and _39_ seconds. This is worse than with _FAST_. 

Result set is for _ORB_:

```
16.4544s, 17.2421s, 29.8826s, 26.3184s, 25.666s, 18.2578s, 39.9755s, 25.8055s, 13.2757s, 10.8056s, 10.4394s, 18.4984s, 8.67498s, 18.7476s, 13.225s
```

In Mid project _HARRIS_ returned only a low number of matches. This results again in outliers and wrong collision times. Due to the low performance we omit the results here. 

Please see Appendix A for a complete list of detectors and descriptors and their ttcs.


## Different Descriptors

All the combinations of detectors and descriptors can be found in Appendix A but one can clearly see that the quality of the result highly depends on the chosen detectors. The descriptors do have an influence but won't turn a good result in a bad result and vice versa. Though, we need to be careful with the selection of the algorithms as some combinations yield in OpenCV errors. See mid term report for further explanation.


# APPENDIX A

Show the time to collision for all possible combinations of detectors and descriptors. Each element in the list represents the ttc in one individual frame.

- detectorType: SHITOMASI - descriptorType: BRISK
```
12.94s, 14.11s, 12.35s, 12.58s, 13.26s, 11.94s, 13.15s, 14.85s, 10.70s, 10.42s, 
```

- detectorType: SHITOMASI - descriptorType: BRIEF
```
14.63s, 9.40s, 13.49s, 12.80s, 13.22s, 13.45s, 12.58s, 13.14s, 10.96s, 11.34s, 8.17s, 9.85s, 
```

- detectorType: SHITOMASI - descriptorType: ORB
```
12.32s, 12.69s, 11.75s, 12.55s, 13.90s, 11.74s, 13.01s, 11.27s, 13.51s, 11.03s, 9.51s, 11.10s, 8.34s, 
```

- detectorType: SHITOMASI - descriptorType: FREAK
```
14.23s, 14.10s, 12.69s, 13.2, 10.66s, 12.5, 12.56s, 10.68s, 10.42s, 10.36s, 
```

- detectorType: HARRIS - descriptorType: BRISK
```
10.90s, -11.47s, 11.57s, 13.64s, 12.99s, 13.19s, 17.62s, nans, -153.9s, 11.69s, 568.32s, -13.62s, 6.71s, -infs, 
```

- detectorType: HARRIS - descriptorType: BRIEF
```
10.90s, -11.47s, 11.57s, 35.38s, 15.24s, 14.27s, 17.62s, -13.44s, 11.69s, 13.43s, -13.62s, 6.76s, 12.58s, 12.83s, 
```

- detectorType: HARRIS - descriptorType: ORB
```
10.90s, -80.85s, 11.57s, 13.64s, 15.24s, 13.49s, 17.62s, 20.58s, 11.21s, 11.69s, 13.43s, -13.62s, 6.71s, -infs, 
```

- detectorType: HARRIS - descriptorType: FREAK
```
9.74s, -11.47s, 12.12s, 35.38s, 13.59s, 13.34s, 12.37s, 10.29s, 11.15s, 11.69s, 0.03s, 6.65s, 25.67s, 
```

- detectorType: FAST - descriptorType: BRISK
```
12.08s, 11.17s, 12.73s, 12.01s, 11.09s, 10.52s, 10.51s, 11.98s, 
```

- detectorType: FAST - descriptorType: BRIEF
```
11.80s, 11.38s, 13.08s, 12.48s, 14.97s, 13.72s, 13.07s, 11.9s, 13.41s, 11.81s, 11.33s, 12.13s, 10.38s, 10.64s, 10.81s, 10.62s, 11.02s, 
```

- detectorType: FAST - descriptorType: ORB
```
11.32s, 11.83s, 15.83s, 12.35s, 13.57s, 11.88s, 12.50s, 11.32s, 9.91s, 11.23s, 10.53s, 10.60s, 10.23s, 9.14s, 11.41s,
```

- detectorType: FAST - descriptorType: FREAK
```
12.47s, 10.29s, 12.95s, 11.45s, 11.82s, 12.11s, 11.85s, 11.13s, 10.45s, 10.65s, 10.9s, 
```

- detectorType: BRISK - descriptorType: BRISK
```
14.20s, 18.59s, 13.91s, 15.15s, 26.77s, 15.54s, 15.84s, 16.71s, 15.57s, 12.54s, 11.70s, 12.15s, 13.10s, 11.04s, 9.01s, 10.36s, 
```

- detectorType: BRISK - descriptorType: BRIEF
```
17.43s, 19.56s, 15.18s, 21.82s, 29.44s, 19.66s, 16.02s, 19.98s, 17.15s, 14.30s, 11.52s, 15.87s, 13.40s, 11.23s, 11.62s, 12.53s, 9.85s, 9.69s, 
```

- detectorType: BRISK - descriptorType: ORB
```
19.03s, 16.03s, 15.66s, 19.86s, 22.96s, 16.59s, 14.96s, 13.42s, 12.64s, 12.37s, 11.83s, 13.22s, 11.23s, 8.71s, 10.58s, 
```

- detectorType: BRISK - descriptorType: FREAK
```
13.93s, 26.33s, 15.43s, 14.24s, 33.67s, 12.81s, 12.87s, 17.56s, 17.36s, 12.80s, 11.16s, 12.43s, 12.26s, 12.62s, 12.91s, 7.96s, 11.13s, 
```

- detectorType: ORB - descriptorType: BRISK
```
18.77s, 15.62s, 21.30s, 67.15s, 12.97s, 18.9s, 10.89s, 12.0s, 7.80s, 47.56s, 9.66s, 9.81s, 15.42s, 21.65s, 
```

- detectorType: ORB - descriptorType: BRIEF
```
16.45s, 17.24s, 29.88s, 26.31s, 25.66s, 18.25s, 39.97s, 25.80s, 13.27s, 10.80s, 10.43s, 18.49s, 8.67s, 18.74s, 13.22s, 
```

- detectorType: ORB - descriptorType: ORB
```
10.12s, 25.55s, 25.77s, 10.76s, 19.90s, 10.31s, 18.65s, 15.55s, 7.86s, 31.50s, 9.59s, 14.47s, 21.50s, 
```

- detectorType: ORB - descriptorType: FREAK
```
12.23s, 24.13s, 12.13s, 35.86s, 8.93s, 13.39s, -infs, 8.34s, 7.03s, 54.22s, -49.00s, 44.57s, 
```

- detectorType: AKAZE - descriptorType: BRISK
```
12.12s, 14.90s, 13.31s, 14.82s, 14.35s, 14.88s, 14.48s, 11.79s, 12.58s, 10.84s, 10.50s, 10.10s, 10.07s, 10.32s, 9.21s, 9.22s, 
```

- detectorType: AKAZE - descriptorType: BRIEF
```
13.88s, 15.75s, 13.44s, 15.77s, 15.34s, 16.06s, 15.25s, 15.53s, 12.65s, 14.18s, 11.91s, 10.7s, 10.04s, 9.50s, 10.59s, 9.40s, 9.35s, 
```

- detectorType: AKAZE - descriptorType: ORB
```
13.55s, 14.19s, 13.24s, 15.61s, 14.83s, 14.48s, 11.92s, 12.96s, 11.19s, 10.07s, 10.57s, 9.23s, 9.09s, 
```

- detectorType: AKAZE - descriptorType: FREAK
```
12.95s, 14.35s, 13.89s, 14.27s, 14.98s, 15.93s, 14.68s, 11.95s, 12.53s, 10.61s, 10.85s, 9.84s, 9.48s, 9.77s, 9.37s, 9.14s, 
```

- detectorType: AKAZE - descriptorType: AKAZE
```
12.70s, 14.28s, 13.03s, 14.73s, 14.98s, 15.32s, 12.07s, 12.70s, 11.18s, 10.73s, 10.83s, 10.56s, 10.45s, 9.26s, 9.10s, 
```

- detectorType: SIFT - descriptorType: BRISK
```
13.42s, 12.36s, 12.35s, 18.48s, 15.04s, 11.05s, 13.08s, 12.17s, 8.81s, 
```

- detectorType: SIFT - descriptorType: BRIEF
```
11.51s, 11.31s, 14.31s, 20.69s, 15.13s, 11.36s, 13.20s, 11.70s, 9.00s, 8.56s, 9.24s, 
```

- detectorType: SIFT - descriptorType: FREAK
```
12.12s, 13.51s, 12.58s, 17.95s, 14.51s, 11.29s, 13.54s, 11.97s, 9.92s, 8.36s
```
