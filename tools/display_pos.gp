p="/tmp/position.dat"

set output 'left_hand.png'
set terminal png
set title "Left Hand"
plot p u 1 w l t "LIP", p u 2 w l t "LMP", p u 3 w l t "LPP", p u 4 w l t "LRP", p u 5 w l t "LTPP", p u 6 w l t "LTPY", p u 7 w l t "RIP", p u 8 w l t "RMP", p u 9 w l t "RPP", p u 10 w l t "RRP", p u 11 w l t "RTPP", p u 12 w l t "RTPY"

set output 'right_hand.png'
set title "Right Hand"
plot p u 13 w l t "LIP", p u 14 w l t "LMP", p u 15 w l t "LPP", p u 16 w l t "LRP", p u 17 w l t "LTPP", p u 18 w l t "LTPY", p u 19 w l t "RIP", p u 20 w l t "RMP", p u 21 w l t "RPP", p u 22 w l t "RRP", p u 23 w l t "RTPP", p u 24 w l t "RTPY"

set output 'left_leg.png'
set title "Left Leg"
plot  p u 28 w l t "lhp", p u 29 w l t "lhr", p u 30 w l t "lhy", p u 31 w l t "lk", p u 25 w l t "lap", p u 26 w l t "lar"

set output 'right_leg.png'
set title "Right Leg"
plot  p u 41 w l t "rhp", p u 42 w l t "rhr", p u 43 w l t "rhy", p u 44 w l t "rk", p u 38 w l t "rap", p u 39 w l t "rar"

set output 'left_arm.png'
set title "Left Arm"
plot  p u 27 w l t "le", p u 32 w l t "lsp", p u 33 w l t "lsr", p u 34 w l t "lsy", p u 35 w l t "lwp", p u 36 w l t "lwr", p u 37 w l t "lwy"

set output 'right_arm.png'
set title "Right Arm"
plot  p u 40 w l t "re", p u 45 w l t "rsp", p u 46 w l t "rsr", p u 47 w l t "rsy", p u 48 w l t "rwp", p u 49 w l t "rwr", p u 50 w l t "rwy"
unset multiplot
