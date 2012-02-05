
#A <- as.matrix(read.csv("m004_r011_b15255_261324_273612.csv",header=FALSE)); # 17:21:30 szep fel
#A <- as.matrix(read.csv("m004_r011_b15255_279756_285900.csv",header=FALSE)); # 17:23:00 szep le
#A <- as.matrix(read.csv("m004_r011_b15255_293068_297164.csv",header=FALSE)); # 17:24:05 szep fel

#A <- as.matrix(read.csv("m004_r011_b15255_310476_318668.csv",header=FALSE)); # 17:25:30 eleje sporol
#A <- as.matrix(read.csv("m004_r011_b15255_343244_345292.csv",header=FALSE)); # 17:28:10 eleje sporol
#A <- as.matrix(read.csv("m004_r011_b15255_357580_361676.csv",header=FALSE)); # 17:29:20 vege sporol

#A <- as.matrix(read.csv("m004_r011_b15255_371916_373964.csv",header=FALSE)); # 17:30:30 eltulzott kocsi
A <- as.matrix(read.csv("m004_r011_b15255_381132_387276.csv",header=FALSE)); # 17:31:15 verseny tempo fel
#A <- as.matrix(read.csv("m004_r011_b15255_391372_394444.csv",header=FALSE)); # 17:32:05 bal melyre

#A <- as.matrix(read.csv("m004_r011_b15255_542924_544357.csv",header=FALSE)); # 17:44:25 idoszinkronizacionak tunik

r <- nrow(A)


x <- c(1:r);
#x <- c(2000:4000);

accx <- c(A[x,7])

plot( accx,type="l",col="red")

accy <- c(A[x,8])
accz <- c(A[x,9])

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
