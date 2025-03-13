p="/tmp/position.dat"

set terminal png

set output 'left_leg.png'
set title "Left Leg"
plot  p u 4 w l t "lhp", p u 5 w l t "lhr", p u 6 w l t "lhy", p u 7 w l t "lk", p u 1 w l t "lap", p u 2 w l t "lar"

set output 'right_leg.png'
set title "Right Leg"
plot  p u 17 w l t "rhp", p u 18 w l t "rhr", p u 19 w l t "rhy", p u 20 w l t "rk", p u 14 w l t "rap", p u 15 w l t "rar"

set output 'left_arm.png'
set title "Left Arm"
plot  p u 3 w l t "le", p u 8 w l t "lsp", p u 9 w l t "lsr", p u 10 w l t "lsy", p u 11 w l t "lwp", p u 12 w l t "lwr", p u 13 w l t "lwy"

set output 'right_arm.png'
set title "Right Arm"
plot  p u 16 w l t "re", p u 21 w l t "rsp", p u 22 w l t "rsr", p u 23 w l t "rsy", p u 24 w l t "rwp", p u 25 w l t "rwr", p u 26 w l t "rwy"
unset multiplot
