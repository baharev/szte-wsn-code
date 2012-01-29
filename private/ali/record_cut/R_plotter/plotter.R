
A <- as.matrix(read.csv("m005_r028_b7686_196797_233661.csv",header=FALSE));

r <- nrow(A)


x <- c(1:r);
x <- c(8000:22000);

accx <- c(A[x,4])

plot( accx,type="l",col="red")

accy <- c(A[x,5])
accz <- c(A[x,6])

lines(accy, type="l", col="green")
lines(accz, type="l", col="blue")

vx <- c( 9950, 9950);
vy <- c(  500, 5000);

lines(vx, vy, type="l", col="black");

vx <- c( 9200, 9200)
vy <- c(  500, 5000)

lines(vx, vy, type="l", col="black")

