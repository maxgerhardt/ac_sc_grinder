#!/usr/bin/env node

/*
  Build `./src/config_map.h` from yaml src
*/

/* eslint-disable no-console*/

const yaml = require('js-yaml').safeLoad;
const path = require('path');
const read = require('fs').readFileSync;
const write = require('fs').writeFileSync;

const config = yaml(read(path.resolve(__dirname, './config.yml')));

//
// Create list of #define-s
//

let entries = '';

function cfg_entry_text(obj, idx) {
  let val = (obj.default || 0.0).toString();

  // Force float reprezentation for C.
  if (!/\./.test(val)) val += '.0';

  return `#define CFG_${obj.name.toUpperCase()}_ADDR ${idx}
#define CFG_${obj.name.toUpperCase()}_DEFAULT ${val}

`;
}

// TODO: use proper iterator with nesting support
Object.keys(config.schema.properties).forEach(key => {
  entries += cfg_entry_text(config.schema.properties[key], key);
});

//
// Create full file content
//

const out = `#ifndef __CONFIG_MAP__
#define __CONFIG_MAP__

// This is autogenerated file, with EEPROM map & defaults.
// Use \`npm run build\` to regenerate.

// Every virtual cell is 4-byte data, float.

${entries}
#endif
`;

//
// Save result
//

write(path.resolve(__dirname, '../src/config_map.h'), out);
//console.log(out);
