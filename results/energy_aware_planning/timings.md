# Deck planning times

`CELL_RESOLUTION=0.1` · `HOP_RADIUS=1.5` · `V_MAX=7.345` · `ROBOT_RADIUS=0.15`

`v_s/V_max` is the fraction of the leg's energy budget a hop consumes.

| scenario | variant | seconds | expansions | edge checks | accepted | hops | path (m) | mean v_s/V_max | max v_s/V_max |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| tall_wall | ballistic | 4.945 | 812 | 167534 | 143712 | 4 | 4.27 | 0.566 | 0.630 |
| tall_wall | baseline | 0.130 | 104 | 21470 | 21427 | 3 | 4.00 | 0.682 | 0.786 |
| tall_narrow_wall | ballistic | 1.174 | 123 | 25573 | 23403 | 3 | 4.00 | 0.625 | 0.693 |
| tall_narrow_wall | baseline | 0.160 | 127 | 26437 | 26437 | 3 | 4.00 | 0.625 | 0.693 |
| barely_jumpable_wall | ballistic | 1.177 | 123 | 25573 | 23403 | 3 | 4.00 | 0.625 | 0.693 |
| barely_jumpable_wall | baseline | 0.166 | 127 | 26437 | 26437 | 3 | 4.00 | 0.625 | 0.693 |
| tall_stairs | ballistic | 49.462 | 10325 | 1810018 | 1582059 | 3 | 3.50 | 0.570 | 0.603 |
| tall_stairs | baseline | 13.144 | 12642 | 2265250 | 2243291 | 3 | 3.50 | 0.576 | 0.603 |
| stairs_with_curb | ballistic | 30.687 | 7431 | 1256718 | 1039224 | 4 | 4.28 | 0.646 | 0.739 |
| stairs_with_curb | baseline | 12.280 | 11971 | 2146077 | 2033307 | 3 | 4.00 | 0.720 | 0.786 |
| slope_crest | ballistic | 31.033 | 6343 | 1082168 | 1075996 | 4 | 4.00 | 0.662 | 0.715 |
| slope_crest | baseline | 5.251 | 5270 | 902568 | 900345 | 4 | 4.00 | 0.650 | 0.704 |
| stairs | ballistic | 0.884 | 89 | 18230 | 16907 | 3 | 4.00 | 0.541 | 0.603 |
| stairs | baseline | 0.133 | 104 | 21470 | 21470 | 3 | 4.00 | 0.682 | 0.786 |

**Total: 150.62 s over 14 runs.**
