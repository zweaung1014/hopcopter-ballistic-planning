# Deck planning times

`CELL_RESOLUTION=0.2` · `HOP_RADIUS=1.5` · `V_MAX=6.000` · `ROBOT_RADIUS=0.1`

`v_s/V_max` is the fraction of the leg's energy budget a hop consumes.

| scenario | variant | seconds | expansions | edge checks | accepted | hops | path (m) | mean v_s/V_max | max v_s/V_max |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| tall_wall | ballistic | 0.201 | 173 | 8778 | 8238 | 3 | 4.26 | 0.622 | 0.644 |
| tall_wall | baseline | 0.008 | 6 | 335 | 335 | 3 | 4.00 | 0.600 | 0.660 |
| tall_narrow_wall | ballistic | 0.175 | 133 | 7141 | 6623 | 4 | 4.00 | 0.499 | 0.630 |
| tall_narrow_wall | baseline | 0.015 | 10 | 569 | 569 | 3 | 4.00 | 0.596 | 0.660 |
| barely_jumpable_wall | ballistic | 0.148 | 117 | 6126 | 5482 | 4 | 4.00 | 0.501 | 0.617 |
| barely_jumpable_wall | baseline | 0.014 | 10 | 569 | 569 | 3 | 4.00 | 0.596 | 0.660 |
| tall_stairs | ballistic | 0.295 | 321 | 14794 | 14794 | 4 | 3.79 | 0.576 | 0.747 |
| tall_stairs | baseline | 0.358 | 417 | 18625 | 18625 | 3 | 3.60 | 0.665 | 0.840 |
| stairs_with_curb | ballistic | 0.314 | 363 | 16342 | 13523 | 3 | 4.00 | 0.700 | 0.782 |
| stairs_with_curb | baseline | 0.346 | 407 | 18148 | 18148 | 3 | 4.00 | 0.691 | 0.840 |
| slope_crest | ballistic | 0.259 | 273 | 12910 | 12220 | 4 | 4.00 | 0.545 | 0.817 |
| slope_crest | baseline | 0.281 | 317 | 13598 | 13598 | 4 | 4.00 | 0.577 | 0.716 |
| stairs | ballistic | 0.037 | 27 | 1457 | 1353 | 4 | 4.00 | 0.496 | 0.703 |
| stairs | baseline | 0.008 | 6 | 335 | 335 | 3 | 4.00 | 0.600 | 0.660 |

**Total: 2.46 s over 14 runs.**
