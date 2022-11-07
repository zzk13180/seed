const fs = require('fs')
const path = require('path')
const less2Json = require('./less-vars-to-json.js')

const source = 'src/theme/globalLessVars/index.less'

less2Json(source)
  .then(result => {
    fs.writeFile(`${path.dirname(source)}/index.json`, JSON.stringify(result), err => {
      if (err) {
        console.log(err)
      }
    })
  })
  .catch(err => {
    console.error(err)
  })
