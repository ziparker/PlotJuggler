// here we can add some common methods/libraries
// for example some quaternion libraries, helpers to calculate angular distance, etc

// source: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles

function quaternionToEulerAngle(x, y, z, w) {

  DEG_TO_RAD = (Math.PI / 180)

  // roll (x-axis rotation)
  t0 = 2.0 * (w * x + y * z);
  t1 = 1.0 - 2.0 * (x * x + y * y);
  X = Math.atan2(t0, t1);

  // pitch (y-axis rotation)
  t2 = 2.0 * (w * y - z * x);
  if(t2 > 1.0) {
    t2 = 1.0;
  } else if(t2 < -1.0){
    t2 = -1.0;
  }
  Y = Math.asin(t2);

  // yaw (z-axis rotation)
  t3 = 2.0 * (w * z + x * y);
  t4 = 1.0 - 2.0 * (y * y + z * z);
  Z = Math.atan2(t3, t4);

  Xdeg = X / DEG_TO_RAD
  Ydeg = Y / DEG_TO_RAD
  Zdeg = Z / DEG_TO_RAD

  return [Xdeg, Ydeg, Zdeg]
}
