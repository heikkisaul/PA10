/* Written by Songyot Piriyakulkit
   For Hirata Lab. PA10 Training
   November 2017*/
struct status {
  double pos;
  double vel;
  double acc;
};
struct path{
  double pos[6];
  double vel[5];
  double acc[4];
  double time;
};
struct params{
  double desPos;
  double desTime;
};
