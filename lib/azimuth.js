/*
 * azimuth2
 * https://github.com/JesusIslam/azimuth
 *
 * Copyright (c) 2017 Andida Syahendar
 * Licensed under the MIT license.
 */

"use strict";

/**
 * Constants
 */

// http://en.wikipedia.org/wiki/Earth_radius
// equatorial radius in meters
const RADIUS_EQUATOR = 6378137.0;
const a = RADIUS_EQUATOR;
const asqr = a*a;

// polar radius in meters
const FLATTENING_DENOM = 298.257223563
const FLATTENING = 1 / FLATTENING_DENOM;
const RADIUS_POLES = RADIUS_EQUATOR*(1-FLATTENING);
const b = RADIUS_POLES;
const bsqr = b*b;

const e = Math.sqrt((asqr - bsqr) / asqr);
const eprime = Math.sqrt((asqr - bsqr) / bsqr);

/**
 * Calculates azimuth, distance and altitude
 *
 * @param {Object} from
 * @param {Object} to
 * @returns {Object}
 */
exports.RAE = function(from, to) {
  let ap, bp, dist, br, theta, azimuth, shadow, altitude;

  ap = locationToPoint(from);
  bp = locationToPoint(to);
  dist = distance(ap, bp);

  // Let's use a trick to calculate azimuth:
  // Rotate the globe so that point A looks like latitude 0, longitude 0.
  // We keep the actual radii calculated based on the oblate geoid,
  // but use angles based on subtraction.
  // Point A will be at x=radius, y=0, z=0.
  // Vector difference B-A will have dz = N/S component, dy = E/W component.

  br = rotateGlobe(to, from, bp.radius);
  theta = Math.atan2(br.z, br.y) * 180.0 / Math.PI;
  azimuth = 90.0 - theta;

  if (azimuth < 0.0) {
    azimuth += 360.0;
  }

  if (azimuth > 360.0) {
    azimuth -= 360.0;
  }

  // Calculate altitude, which is the angle above the horizon of B as seen
  // from A. Almost always, B will actually be below the horizon, so the altitude
  // will be negative.
  shadow = Math.sqrt((br.y * br.y) + (br.z * br.z));
  altitude = Math.atan2(br.x - ap.radius, shadow) * 180.0 / Math.PI;

  return { range: dist, azimuth: azimuth, elevation: altitude };
};

/**
 * Calculate RAE to LLA
 * 
 * @param {Object} siteLLA
 * @param {Object} RAE
 * @returns {Object}
 */
exports.RAEtoLLA = function(siteLLA, RAE) {
  let SEZ = RAEtoSEZ(siteLLA, RAE);
  let ECR = SEZtoECR(siteLLA, SEZ);
  let LLA = ECRtoLLA(ECR);

  return LLA;
};

function RAEtoSEZ(LLA, RAE) {
  let range = RAE.range;
  let azimuth = RAE.azimuth;
  let elevation = RAE.elevation;

  let lat = degreeToRadian(LLA.latitude);
  let lon = degreeToRadian(LLA.longitude);
  let altitude = LLA.altitude;

  let slat = Math.sin(lat);
  let slon = Math.sin(lon);
  let clat = Math.cos(lat);
  let clon = Math.cos(lon);

  azimuth = degreeToRadian(azimuth);
  elevation = degreeToRadian(elevation);

  let SEZ = {
    south: -range * Math.cos(elevation) * Math.cos(azimuth),
    east: range * Math.cos(elevation) * Math.sin(azimuth),
    zenith: range * Math.sin(elevation)
  };

  return SEZ;
}

function SEZtoECR(LLA, SEZ) {
  let ECR = locationToPoint(LLA);

  let lat = degreeToRadian(LLA.latitude);
  let lon = degreeToRadian(LLA.longitude);
  let altitude = LLA.altitude;

  let south = SEZ.south;
  let east = SEZ.east;
  let zenith = SEZ.zenith;

  let slat = Math.sin(lat);
  let slon = Math.sin(lon);
  let clat = Math.cos(lat);
  let clon = Math.cos(lon);

  let targetECR = {
    x: (slat * clon * south) + (-slon * east) + (clat * clon * zenith) + ECR.x,
    y: (slat * slon * south) + (clon * east) + (clat * slon * zenith) + ECR.y,
    z: (-clat * south) + (slat * zenith) + ECR.z,
    radius: ECR.radius
  };

  return targetECR;
}

function ECRtoLLA(ECR) {
  let X = ECR.x;
  let Y = ECR.y;
  let Z = ECR.z;

  //Auxiliary values first
  let p = Math.sqrt(X*X + Y*Y);
  let theta = Math.atan((Z*a) / (p*b));

  let sintheta = Math.sin(theta);
  let costheta = Math.cos(theta);

  let num = Z + eprime * eprime * b * sintheta * sintheta * sintheta;
  let denom = p - e * e * a * costheta * costheta * costheta;

  //Now calculate LLA
  let latitude  = Math.atan(num / denom);
  let longitude = Math.atan(Y / X);
  let N = getN(latitude);
  let altitude  = (p / Math.cos(latitude)) - N;

  if (X < 0 && Y < 0) {
    longitude = longitude - Math.PI;
  }

  if (X < 0 && Y > 0) {
    longitude = longitude + Math.PI;
  }
  
  latitude = radianToDegree(latitude);
  longitude = radianToDegree(longitude);

  return {
    latitude: latitude,
    longitude: longitude,
    altitude: altitude
  };
}

function getN(latitude) {
    var sinlatitude = Math.sin(latitude);
    var denom = Math.sqrt(1-e*e*sinlatitude*sinlatitude);
    var N = a / denom;
    return N;
}

/**
 * Determines radius
 *
 * @param {Number} latRadians
 * @returns {Number}
 */
function getRadius(latRadians) {
  let cos, sin, t1, t2, t3, t4;

  cos = Math.cos(latRadians);
  sin = Math.sin(latRadians);
  t1 = asqr * cos;
  t2 = bsqr * sin;
  t3 = a * cos;
  t4 = b * sin;

  return Math.sqrt((t1 * t1 + t2 * t2) / (t3 * t3 + t4 * t4));
}

/**
 * Converts lat, lng, elv to x, y, z
 *
 * @param {Object} loc
 * @returns {Object}
 */
function locationToPoint(loc) {
  let lat, lng, radius, cosLat, sinLat, cosLng, sinLng, x, y, z;

  lat = loc.latitude * Math.PI / 180.0;
  lng = loc.longitude * Math.PI / 180.0;
  radius = loc.altitude + getRadius(lat);
  cosLng = Math.cos(lng);
  sinLng = Math.sin(lng);
  cosLat = Math.cos(lat);
  sinLat = Math.sin(lat);
  x = cosLng * cosLat * radius;
  y = sinLng * cosLat * radius;
  z = sinLat * radius;

  return { x: x, y: y, z: z, radius: radius };
}

/**
 * Calculates distance between two points in 3d space
 *
 * @param {Object} ap
 * @param {Object} bp
 * @returns {Number}
 */
function distance(ap, bp) {
  let dx, dy, dz;

  dx = ap.x - bp.x;
  dy = ap.y - bp.y;
  dz = ap.z - bp.z;

  return Math.sqrt(dx * dx + dy * dy + dz * dz);
}

/**
 * Gets rotated point
 *
 * @param {Object} b
 * @param {Object} a
 * @param {Number} bRadius
 * @returns {Object}
 */
function rotateGlobe(b, a, bRadius) {
  let br, brp, alat, acos, asin, bx, by, bz;

  // Get modified coordinates of 'b' by rotating the globe so
  // that 'a' is at lat=0, lng=0
  br = { latitude: b.latitude, longitude: (b.longitude - a.longitude), altitude: b.altitude };
  brp = locationToPoint(br);

  // scale all the coordinates based on the original, correct geoid radius
  brp.x *= (bRadius / brp.radius);
  brp.y *= (bRadius / brp.radius);
  brp.z *= (bRadius / brp.radius);

  // restore actual geoid-based radius calculation
  brp.radius = bRadius;

  // Rotate brp cartesian coordinates around the z-axis by a.longitude degrees,
  // then around the y-axis by a.latitude degrees.
  // Though we are decreasing by a.latitude degrees, as seen above the y-axis,
  // this is a positive (counterclockwise) rotation
  // (if B's longitude is east of A's).
  // However, from this point of view the x-axis is pointing left.
  // So we will look the other way making the x-axis pointing right, the z-axis
  // pointing up, and the rotation treated as negative.
  alat = -a.latitude * Math.PI / 180.0;
  acos = Math.cos(alat);
  asin = Math.sin(alat);

  bx = (brp.x * acos) - (brp.z * asin);
  by = brp.y;
  bz = (brp.x * asin) + (brp.z * acos);

  return { x: bx, y: by, z: bz };
}

function degreeToRadian(d) {
  return d * (Math.PI / 180);
}

function radianToDegree(r) {
  return r * (180 / Math.PI);
}
