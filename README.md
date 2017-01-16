Determines azimuth (compass direction) & range of the second point (B) as seen from the first point (A), given latitude, longitude & altitude of two points on the Earth.
Also determine latitude, longitude, & altitude from given range, azimuth, and elevation.

## Original Author
Moulded from [Don Cross](http://cosinekitty.com/)'s [Azimuth/Distance Calculator](http://cosinekitty.com/compass.html)
Forked from [Debjeet Biswas](http://github.com/vxtindia/azimuth)

## Getting Started
Install the module with: `npm install azimuth2`

```javascript
let calculate = require('azimuth2');
let rae = calculate.RAE({
  latitude: 18.513929,
  longitude: 73.924475,
  altitude: 561.9
}, {
  latitude: 18.513964,
  longitude: 73.924471,
  altitude: 562
}); // "{ range: 3.9191090699705464, azimuth: 353.8149364508667, elevation: 1.3478271564744548 }"

let lla = calculate.RAEtoLLA(rae);
```

## Tests
To run tests

````
npm test
````

or

````
grunt nodeunit
````

## License
Copyright (c) 2017 Andida Syahendar
Licensed under the MIT license.