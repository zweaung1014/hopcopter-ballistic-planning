# Deck planning times

`CELL_RESOLUTION=0.1` · `HOP_RADIUS=1.5` · `V_MAX=4.852` · `ROBOT_RADIUS=0.2`

`v_s/V_max` is the fraction of the leg's energy budget a hop consumes.

| scenario | variant | seconds | expansions | edge checks | accepted | hops | path (m) | mean v_s/V_max | max v_s/V_max |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| tall_wall | ballistic | 1.720 | 256 | 51756 | 43235 | 4 | 4.39 | 0.789 | 0.938 |
| tall_wall | baseline | 0.032 | 78 | 16147 | 15276 | 3 | 4.00 | 0.814 | 1.000 |
| tall_narrow_wall | ballistic | 0.569 | 78 | 16147 | 14171 | 3 | 4.00 | 0.785 | 0.913 |
| tall_narrow_wall | baseline | 0.041 | 99 | 20682 | 20630 | 3 | 4.00 | 0.808 | 0.955 |
| barely_jumpable_wall | ballistic | 0.520 | 76 | 15715 | 12919 | 3 | 4.00 | 0.832 | 0.970 |
| barely_jumpable_wall | baseline | 0.041 | 99 | 20682 | 19686 | 3 | 4.00 | 0.823 | 1.000 |
| tall_stairs | ballistic | 6.547 | 1196 | 205603 | 165669 | 3 | 3.50 | 0.822 | 0.902 |
| tall_stairs | baseline | 0.556 | 1650 | 295234 | 289606 | 3 | 3.50 | 0.883 | 0.938 |
| stairs_with_curb | ballistic | 5.478 | 1035 | 173618 | 137130 | 5 | 4.00 | 0.687 | 0.950 |
| stairs_with_curb | baseline | 0.531 | 1550 | 275030 | 261189 | 3 | 4.00 | 0.921 | 1.000 |
| slope_crest | ballistic | 7.355 | 1053 | 182480 | 99186 | 4 | 4.00 | 0.675 | 0.861 |
| slope_crest | baseline | 0.273 | 822 | 143113 | 142847 | 3 | 4.00 | 0.798 | 0.953 |
| stairs | ballistic | 0.718 | 96 | 19674 | 17317 | 4 | 4.02 | 0.633 | 0.819 |
| stairs | baseline | 0.033 | 78 | 16147 | 16147 | 3 | 4.00 | 0.814 | 1.000 |

**Total: 24.41 s over 14 runs.**
