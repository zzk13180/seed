import{l as i}from"./preact.module.c9e75aee.js";var l,u,s,y,v=0,b=[],h=[],E=i.__b,V=i.__r,F=i.diffed,g=i.__c,A=i.unmount;function p(_,o){i.__h&&i.__h(u,_,v||o),v=0;var t=u.__H||(u.__H={__:[],__h:[]});return _>=t.__.length&&t.__.push({__V:h}),t.__[_]}function B(_){return v=1,C(T,_)}function C(_,o,t){var r=p(l++,2);if(r.t=_,!r.__c&&(r.__=[t?t(o):T(void 0,o),function(e){var f=r.__N?r.__N[0]:r.__[0],a=r.t(f,e);f!==a&&(r.__N=[a,r.__[1]],r.__c.setState({}))}],r.__c=u,!u.u)){u.u=!0;var n=u.shouldComponentUpdate;u.shouldComponentUpdate=function(e,f,a){if(!r.__c.__H)return!0;var d=r.__c.__H.__.filter(function(c){return c.__c});if(d.every(function(c){return!c.__N}))return!n||n.call(this,e,f,a);var N=!1;return d.forEach(function(c){if(c.__N){var x=c.__[0];c.__=c.__N,c.__N=void 0,x!==c.__[0]&&(N=!0)}}),!(!N&&r.__c.props===e)&&(!n||n.call(this,e,f,a))}}return r.__N||r.__}function O(_,o){var t=p(l++,3);!i.__s&&k(t.__H,o)&&(t.__=_,t.i=o,u.__H.__h.push(t))}function P(_){return v=5,U(function(){return{current:_}},[])}function U(_,o){var t=p(l++,7);return k(t.__H,o)?(t.__V=_(),t.i=o,t.__h=_,t.__V):t.__}function j(){for(var _;_=b.shift();)if(_.__P&&_.__H)try{_.__H.__h.forEach(m),_.__H.__h.forEach(H),_.__H.__h=[]}catch(o){_.__H.__h=[],i.__e(o,_.__v)}}i.__b=function(_){typeof _.type!="function"||_.__m||_.__===null?_.__m||(_.__m=_.__&&_.__.__m?_.__.__m:""):_.__m=(_.__&&_.__.__m?_.__.__m:"")+(_.__&&_.__.__k?_.__.__k.indexOf(_):0),u=null,E&&E(_)},i.__r=function(_){V&&V(_),l=0;var o=(u=_.__c).__H;o&&(s===u?(o.__h=[],u.__h=[],o.__.forEach(function(t){t.__N&&(t.__=t.__N),t.__V=h,t.__N=t.i=void 0})):(o.__h.forEach(m),o.__h.forEach(H),o.__h=[])),s=u},i.diffed=function(_){F&&F(_);var o=_.__c;o&&o.__H&&(o.__H.__h.length&&(b.push(o)!==1&&y===i.requestAnimationFrame||((y=i.requestAnimationFrame)||w)(j)),o.__H.__.forEach(function(t){t.i&&(t.__H=t.i),t.__V!==h&&(t.__=t.__V),t.i=void 0,t.__V=h})),s=u=null},i.__c=function(_,o){o.some(function(t){try{t.__h.forEach(m),t.__h=t.__h.filter(function(r){return!r.__||H(r)})}catch(r){o.some(function(n){n.__h&&(n.__h=[])}),o=[],i.__e(r,t.__v)}}),g&&g(_,o)},i.unmount=function(_){A&&A(_);var o,t=_.__c;t&&t.__H&&(t.__H.__.forEach(function(r){try{m(r)}catch(n){o=n}}),t.__H=void 0,o&&i.__e(o,t.__v))};var q=typeof requestAnimationFrame=="function";function w(_){var o,t=function(){clearTimeout(r),q&&cancelAnimationFrame(o),setTimeout(_)},r=setTimeout(t,100);q&&(o=requestAnimationFrame(t))}function m(_){var o=u,t=_.__c;typeof t=="function"&&(_.__c=void 0,t()),u=o}function H(_){var o=u;_.__c=_.__(),u=o}function k(_,o){return!_||_.length!==o.length||o.some(function(t,r){return t!==_[r]})}function T(_,o){return typeof o=="function"?o(_):o}export{U as F,P as _,O as h,B as p};
