# (c) 2021 Philipp Ruppel

python src/tractor/plot.py \
loss_gncg.csv "Gauss-Newton+CG" \
loss_gd_1.csv "BackProp lr=1" \
loss_gd_01.csv "BackProp lr=0.1" \
loss_gd_001.csv "BackProp lr=0.01" \
loss_gd_0001.csv "BackProp lr=0.001"
