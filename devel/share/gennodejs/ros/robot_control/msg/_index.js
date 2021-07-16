
"use strict";

let fsrInput = require('./fsrInput.js');
let tactile = require('./tactile.js');
let newtactile = require('./newtactile.js');
let coord = require('./coord.js');
let accelerometr = require('./accelerometr.js');
let state = require('./state.js');
let rigid = require('./rigid.js');
let contact = require('./contact.js');

module.exports = {
  fsrInput: fsrInput,
  tactile: tactile,
  newtactile: newtactile,
  coord: coord,
  accelerometr: accelerometr,
  state: state,
  rigid: rigid,
  contact: contact,
};
