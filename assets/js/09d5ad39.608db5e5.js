(self.webpackChunkdocs=self.webpackChunkdocs||[]).push([[3030],{3905:function(e,t,n){"use strict";n.d(t,{Zo:function(){return s},kt:function(){return m}});var r=n(7294);function o(e,t,n){return t in e?Object.defineProperty(e,t,{value:n,enumerable:!0,configurable:!0,writable:!0}):e[t]=n,e}function a(e,t){var n=Object.keys(e);if(Object.getOwnPropertySymbols){var r=Object.getOwnPropertySymbols(e);t&&(r=r.filter((function(t){return Object.getOwnPropertyDescriptor(e,t).enumerable}))),n.push.apply(n,r)}return n}function i(e){for(var t=1;t<arguments.length;t++){var n=null!=arguments[t]?arguments[t]:{};t%2?a(Object(n),!0).forEach((function(t){o(e,t,n[t])})):Object.getOwnPropertyDescriptors?Object.defineProperties(e,Object.getOwnPropertyDescriptors(n)):a(Object(n)).forEach((function(t){Object.defineProperty(e,t,Object.getOwnPropertyDescriptor(n,t))}))}return e}function c(e,t){if(null==e)return{};var n,r,o=function(e,t){if(null==e)return{};var n,r,o={},a=Object.keys(e);for(r=0;r<a.length;r++)n=a[r],t.indexOf(n)>=0||(o[n]=e[n]);return o}(e,t);if(Object.getOwnPropertySymbols){var a=Object.getOwnPropertySymbols(e);for(r=0;r<a.length;r++)n=a[r],t.indexOf(n)>=0||Object.prototype.propertyIsEnumerable.call(e,n)&&(o[n]=e[n])}return o}var l=r.createContext({}),p=function(e){var t=r.useContext(l),n=t;return e&&(n="function"==typeof e?e(t):i(i({},t),e)),n},s=function(e){var t=p(e.components);return r.createElement(l.Provider,{value:t},e.children)},u={inlineCode:"code",wrapper:function(e){var t=e.children;return r.createElement(r.Fragment,{},t)}},d=r.forwardRef((function(e,t){var n=e.components,o=e.mdxType,a=e.originalType,l=e.parentName,s=c(e,["components","mdxType","originalType","parentName"]),d=p(n),m=o,f=d["".concat(l,".").concat(m)]||d[m]||u[m]||a;return n?r.createElement(f,i(i({ref:t},s),{},{components:n})):r.createElement(f,i({ref:t},s))}));function m(e,t){var n=arguments,o=t&&t.mdxType;if("string"==typeof e||o){var a=n.length,i=new Array(a);i[0]=d;var c={};for(var l in t)hasOwnProperty.call(t,l)&&(c[l]=t[l]);c.originalType=e,c.mdxType="string"==typeof e?e:o,i[1]=c;for(var p=2;p<a;p++)i[p]=n[p];return r.createElement.apply(null,i)}return r.createElement.apply(null,n)}d.displayName="MDXCreateElement"},4909:function(e,t,n){"use strict";n.r(t),n.d(t,{frontMatter:function(){return i},metadata:function(){return c},toc:function(){return l},default:function(){return s}});var r=n(2122),o=n(9756),a=(n(7294),n(3905)),i={},c={unversionedId:"examples",id:"examples",isDocsHomePage:!1,title:"Examples",description:"Moving and Rotating",source:"@site/docs/examples.md",sourceDirName:".",slug:"/examples",permalink:"/fido/docs/examples",editUrl:"https://github.com/hojulian/fido/edit/documentation/docs/docs/examples.md",version:"current",frontMatter:{},sidebar:"mainSidebar",previous:{title:"Getting Started",permalink:"/fido/docs/getting-started"},next:{title:"Concepts",permalink:"/fido/docs/concepts"}},l=[{value:"Moving and Rotating",id:"moving-and-rotating",children:[]},{value:"Physical control",id:"physical-control",children:[]}],p={toc:l};function s(e){var t=e.components,n=(0,o.Z)(e,["components"]);return(0,a.kt)("wrapper",(0,r.Z)({},p,n,{components:t,mdxType:"MDXLayout"}),(0,a.kt)("h2",{id:"moving-and-rotating"},"Moving and Rotating"),(0,a.kt)("pre",null,(0,a.kt)("code",{parentName:"pre",className:"language-python"},'from fido.robot import Turtlebot3\nfrom fido.simulation import Gazebo, Simulation\nfrom fido.world import RaceTrack\n\nrobot = Turtlebot3("bot_01")\nworld = RaceTrack()\nworld.add(robot, x=0, y=0, z=0)\n\nsim = Simulation(\n    simulator=Gazebo(gui=True),\n    world=world,\n)\n\nsim.start()\n\n# Move at speed 2.0 for 5.0s\nrobot.move(speed=2.0, duration=5.0)\nrobot.stop()\n\n# Rotate for 45 degrees.\nrobot.rotate(angle=45.0, speed=2.0)\n\nsim.stop()\nsim.destroy()\n')),(0,a.kt)("h2",{id:"physical-control"},"Physical control"),(0,a.kt)("pre",null,(0,a.kt)("code",{parentName:"pre",className:"language-python"},'from fido.robot import Turtlebot3\n\nrobot = Turtlebot3("bot_01", physical=True)\nrobot.connect(host="127.0.0.1", port=9090)\n\n# Move at speed 2.0 for 5.0s\nrobot.move(speed=2.0, duration=5.0)\nrobot.stop()\n')))}s.isMDXComponent=!0}}]);