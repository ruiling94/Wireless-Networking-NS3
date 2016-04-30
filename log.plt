set terminal postscript
set terminal png size 640,480
set xrange [40:100]
set yrange [0:1]
set output "LogDistancePropagationLossModel.png"
set title "Distance vs Throughput (n=1,LogDistancePropagationLossModel)"
set xlabel "Distance(m)"
set ylabel "Throughput(Mbps)"
plot 'log3.dat' using 1:2 title "Average Throughput" with line
