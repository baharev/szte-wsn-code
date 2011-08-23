generic configuration ReadClientC()
{
  provides {
    interface Read<uint16_t>;
  }
}
implementation
{
  #define UQ_XYZ_RESOURCE "Xyz.ReadResource"
  components new ArbitratedReadC(uint16_t), new FcfsArbiterC(UQ_BMA180_RESOURCE),
             new StdControlPowerManagerC(), ReadClientP, DriverC;
  
  Read=ArbitratedReadC[unique(UQ_XYZ_RESOURCE)];
  
  StdControlPowerManagerC.ResourceDefaultOwner -> FcfsArbiterC;
  StdControlPowerManagerC.Arbiterinfo -> FcfsArbiterC;
  StdControlPowerManagerC.StdControl-> DriverC
  
  ArbitratedReadC.Resource=FcfsArbiterC;
  ArbitratedReadC.Service=ReadClientP;
  
  ReadClientP.ActualRead->DriverC;
}