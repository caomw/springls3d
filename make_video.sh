avconv -r 30 -start_number 1 -i enright%08d_composite.png -b:v 16000k enright.mp4
totem enright.mp4&
