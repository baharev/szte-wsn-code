#A <- as.matrix(read.csv("m005_r028_b7686_196797_233661.csv",header=FALSE));
#A <- as.matrix(read.csv("m005_r037_b52660_260096_272384.csv",header=FALSE)); # 17:21:30 szep fel
#A <- as.matrix(read.csv("m005_r037_b52660_278528_284671.csv",header=FALSE)); # 17:23:00 szep le
#A <- as.matrix(read.csv("m005_r037_b52660_291839_295935.csv",header=FALSE)); # 17:24:05 szep fel
#A <- as.matrix(read.csv("m005_r037_b52660_309247_317439.csv",header=FALSE)); # 17:25:30 eleje sporol
#A <- as.matrix(read.csv("m005_r037_b52660_342015_344063.csv",header=FALSE)); # 17:28:10 eleje sporol
#A <- as.matrix(read.csv("m005_r037_b52660_356351_360447.csv",header=FALSE)); # 17:29:20 vege sporol
#A <- as.matrix(read.csv("m005_r037_b52660_370687_372735.csv",header=FALSE)); # 17:30:30 eltulzott kocsi
A <- as.matrix(read.csv("m005_r037_b52660_379903_386047.csv",header=FALSE)); # 17:31:15 verseny tempo fel
#A <- as.matrix(read.csv("m005_r037_b52660_390143_393215.csv",header=FALSE)); # 17:32:05 bal melyre
#A <- as.matrix(read.csv("m005_r037_b52660_541695_543129.csv",header=FALSE)); # 17:44:25 idoszinkronizacionak tunik

r <- nrow(A)
#rows <- c(400:1800) #szep le oldal forog
#rows <- c(600:2000) #bal melyre
#rows <- c(1:r)
rows <- c(200:2400)

TIME   <- 2
ACC_X  <- 4
ACC_Y  <- 5
GYRO_X <- 7
GYRO_Y <- 8
GYRO_Z <- 9

T_LEN  <- 16
time <- c((A[rows, TIME] - A[1,TIME])/32768.0)

y <- c(A[rows,ACC_X])

plot( time, y, type="l", col="red", 
      main="Hajó gyorsulás, oldalnézet", xlab="Idő (másodperc)", ylab="ADC egység",
      ylim=c(1500,3000), xaxp = c(0, T_LEN, T_LEN))


#ylim=c(2250,2500) GYRO_X
#ylim=c(2600,2800) GYRO_Z

#x <- c(1:r);
#x <- c(2000:6000);
#x <- c(A[c(1:r),2]/32.768)
#ylim=c(2600,2800)

#plot( accx,type="l",col="red")

# accy <- c(A[x,8])
# accz <- c(A[x,9])
# 
# lines(accy, type="l", col="green")
# lines(accz, type="l", col="blue")
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