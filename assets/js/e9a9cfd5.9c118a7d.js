(self.webpackChunkdocs=self.webpackChunkdocs||[]).push([[3047],{3905:function(e,r,t){"use strict";t.d(r,{Zo:function(){return s},kt:function(){return f}});var o=t(7294);function n(e,r,t){return r in e?Object.defineProperty(e,r,{value:t,enumerable:!0,configurable:!0,writable:!0}):e[r]=t,e}function a(e,r){var t=Object.keys(e);if(Object.getOwnPropertySymbols){var o=Object.getOwnPropertySymbols(e);r&&(o=o.filter((function(r){return Object.getOwnPropertyDescriptor(e,r).enumerable}))),t.push.apply(t,o)}return t}function c(e){for(var r=1;r<arguments.length;r++){var t=null!=arguments[r]?arguments[r]:{};r%2?a(Object(t),!0).forEach((function(r){n(e,r,t[r])})):Object.getOwnPropertyDescriptors?Object.defineProperties(e,Object.getOwnPropertyDescriptors(t)):a(Object(t)).forEach((function(r){Object.defineProperty(e,r,Object.getOwnPropertyDescriptor(t,r))}))}return e}function l(e,r){if(null==e)return{};var t,o,n=function(e,r){if(null==e)return{};var t,o,n={},a=Object.keys(e);for(o=0;o<a.length;o++)t=a[o],r.indexOf(t)>=0||(n[t]=e[t]);return n}(e,r);if(Object.getOwnPropertySymbols){var a=Object.getOwnPropertySymbols(e);for(o=0;o<a.length;o++)t=a[o],r.indexOf(t)>=0||Object.prototype.propertyIsEnumerable.call(e,t)&&(n[t]=e[t])}return n}var i=o.createContext({}),d=function(e){var r=o.useContext(i),t=r;return e&&(t="function"==typeof e?e(r):c(c({},r),e)),t},s=function(e){var r=d(e.components);return o.createElement(i.Provider,{value:r},e.children)},p={inlineCode:"code",wrapper:function(e){var r=e.children;return o.createElement(o.Fragment,{},r)}},u=o.forwardRef((function(e,r){var t=e.components,n=e.mdxType,a=e.originalType,i=e.parentName,s=l(e,["components","mdxType","originalType","parentName"]),u=d(t),f=n,m=u["".concat(i,".").concat(f)]||u[f]||p[f]||a;return t?o.createElement(m,c(c({ref:r},s),{},{components:t})):o.createElement(m,c({ref:r},s))}));function f(e,r){var t=arguments,n=r&&r.mdxType;if("string"==typeof e||n){var a=t.length,c=new Array(a);c[0]=u;var l={};for(var i in r)hasOwnProperty.call(r,i)&&(l[i]=r[i]);l.originalType=e,l.mdxType="string"==typeof e?e:n,c[1]=l;for(var d=2;d<a;d++)c[d]=t[d];return o.createElement.apply(null,c)}return o.createElement.apply(null,t)}u.displayName="MDXCreateElement"},614:function(e,r,t){"use strict";t.r(r),t.d(r,{frontMatter:function(){return c},metadata:function(){return l},toc:function(){return i},default:function(){return s}});var o=t(2122),n=t(9756),a=(t(7294),t(3905)),c={sidebar_label:"racetrack",title:"fido.world.racetrack"},l={unversionedId:"reference/fido/world/racetrack",id:"reference/fido/world/racetrack",isDocsHomePage:!1,title:"fido.world.racetrack",description:"RaceTrack Objects",source:"@site/docs/reference/fido/world/racetrack.md",sourceDirName:"reference/fido/world",slug:"/reference/fido/world/racetrack",permalink:"/fido/docs/reference/fido/world/racetrack",editUrl:"https://github.com/hojulian/fido/edit/documentation/docs/docs/reference/fido/world/racetrack.md",version:"current",sidebar_label:"racetrack",frontMatter:{sidebar_label:"racetrack",title:"fido.world.racetrack"},sidebar:"apiSidebar",previous:{title:"fido.simulation.simulator",permalink:"/fido/docs/reference/fido/simulation/simulator"},next:{title:"fido.world.world",permalink:"/fido/docs/reference/fido/world/world"}},i=[{value:"RaceTrack Objects",id:"racetrack-objects",children:[]}],d={toc:i};function s(e){var r=e.components,t=(0,n.Z)(e,["components"]);return(0,a.kt)("wrapper",(0,o.Z)({},d,t,{components:r,mdxType:"MDXLayout"}),(0,a.kt)("h2",{id:"racetrack-objects"},"RaceTrack Objects"),(0,a.kt)("pre",null,(0,a.kt)("code",{parentName:"pre",className:"language-python"},"class RaceTrack(World)\n")),(0,a.kt)("p",null,"Represent a RaceTrack world."),(0,a.kt)("p",null,"For details about this world, see:\n",(0,a.kt)("a",{parentName:"p",href:"https://github.com/aws-robotics/aws-robomaker-racetrack-world"},"https://github.com/aws-robotics/aws-robomaker-racetrack-world"),"."),(0,a.kt)("h4",{id:"add"},"add"),(0,a.kt)("pre",null,(0,a.kt)("code",{parentName:"pre",className:"language-python"}," | add(robot, x=-4.25, y=-15.0, z=0.0)\n")),(0,a.kt)("p",null,"Add a robot to the world."),(0,a.kt)("p",null,"Internally, this is converted into a gazebo_ros spawn_model call."),(0,a.kt)("h4",{id:"remove"},"remove"),(0,a.kt)("pre",null,(0,a.kt)("code",{parentName:"pre",className:"language-python"}," | remove(robot)\n")),(0,a.kt)("p",null,"Remove a robot from the world."),(0,a.kt)("p",null,"Internally, this is converted into a gazebo_ros delete_model call."),(0,a.kt)("h4",{id:"export_files"},"export","_","files"),(0,a.kt)("pre",null,(0,a.kt)("code",{parentName:"pre",className:"language-python"}," | export_files(path, package, rosbridge_port)\n")),(0,a.kt)("p",null,"Export files to a given file."),(0,a.kt)("p",null,"Internally, .rosinstall file is exported to the root of the directory.\nThe launch file is exported to $PATH/src/$PACKAGE/launch."))}s.isMDXComponent=!0}}]);