
#A <- as.matrix(read.csv("m005_r028_b7686_196797_233661.csv",header=FALSE));

#A <- as.matrix(read.csv("m005_r037_b52660_260096_272384.csv",header=FALSE)); # 17:21:30 szep fel
#A <- as.matrix(read.csv("m005_r037_b52660_278528_284671.csv",header=FALSE)); # 17:23:00 szep le
#A <- as.matrix(read.csv("m005_r037_b52660_291839_295935.csv",header=FALSE)); # 17:24:05 szep fel
#A <- as.matrix(read.csv("m005_r037_b52660_309247_317439.csv",header=FALSE)); # 17:25:30 eleje sporol
#A <- as.matrix(read.csv("m005_r037_b52660_342015_344063.csv",header=FALSE)); # 17:28:10 eleje sporol
A <- as.matrix(read.csv("m005_r037_b52660_356351_360447.csv",header=FALSE)); # 17:29:20 vege sporol
#A <- as.matrix(read.csv("m005_r037_b52660_370687_372735.csv",header=FALSE)); # 17:30:30 eltulzott kocsi
#A <- as.matrix(read.csv("m005_r037_b52660_379903_386047.csv",header=FALSE)); # 17:31:15 verseny tempo fel
#A <- as.matrix(read.csv("m005_r037_b52660_390143_393215.csv",header=FALSE)); # 17:32:05 bal melyre
#A <- as.matrix(read.csv("m005_r037_b52660_541695_543129.csv",header=FALSE)); # 17:44:25 idoszinkronizacionak tunik

r <- nrow(A)


x <- c(1:r);
#x <- c(8000:22000);

accx <- c(A[x,4])

plot( accx,type="l",col="red")

accy <- c(A[x,5])
accz <- c(A[x,6])

lines(accy, type="l", col="green")
lines(accz, type="l", col="blue")
# 
# vx <- c( 9950, 9950);
# vy <- c(  500, 5000);
# 
# lines(vx, vy, type="l", col="black");
# 
# vx <- c( 9200, 9200)
# vy <- c(  500, 5000)
# 
# lines(vx, vy, type="l", col="black")
# 
