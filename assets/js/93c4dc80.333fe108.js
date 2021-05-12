(self.webpackChunkdocs=self.webpackChunkdocs||[]).push([[2462],{3905:function(r,e,t){"use strict";t.d(e,{Zo:function(){return d},kt:function(){return m}});var n=t(7294);function o(r,e,t){return e in r?Object.defineProperty(r,e,{value:t,enumerable:!0,configurable:!0,writable:!0}):r[e]=t,r}function a(r,e){var t=Object.keys(r);if(Object.getOwnPropertySymbols){var n=Object.getOwnPropertySymbols(r);e&&(n=n.filter((function(e){return Object.getOwnPropertyDescriptor(r,e).enumerable}))),t.push.apply(t,n)}return t}function i(r){for(var e=1;e<arguments.length;e++){var t=null!=arguments[e]?arguments[e]:{};e%2?a(Object(t),!0).forEach((function(e){o(r,e,t[e])})):Object.getOwnPropertyDescriptors?Object.defineProperties(r,Object.getOwnPropertyDescriptors(t)):a(Object(t)).forEach((function(e){Object.defineProperty(r,e,Object.getOwnPropertyDescriptor(t,e))}))}return r}function l(r,e){if(null==r)return{};var t,n,o=function(r,e){if(null==r)return{};var t,n,o={},a=Object.keys(r);for(n=0;n<a.length;n++)t=a[n],e.indexOf(t)>=0||(o[t]=r[t]);return o}(r,e);if(Object.getOwnPropertySymbols){var a=Object.getOwnPropertySymbols(r);for(n=0;n<a.length;n++)t=a[n],e.indexOf(t)>=0||Object.prototype.propertyIsEnumerable.call(r,t)&&(o[t]=r[t])}return o}var s=n.createContext({}),p=function(r){var e=n.useContext(s),t=e;return r&&(t="function"==typeof r?r(e):i(i({},e),r)),t},d=function(r){var e=p(r.components);return n.createElement(s.Provider,{value:e},r.children)},c={inlineCode:"code",wrapper:function(r){var e=r.children;return n.createElement(n.Fragment,{},e)}},u=n.forwardRef((function(r,e){var t=r.components,o=r.mdxType,a=r.originalType,s=r.parentName,d=l(r,["components","mdxType","originalType","parentName"]),u=p(t),m=o,f=u["".concat(s,".").concat(m)]||u[m]||c[m]||a;return t?n.createElement(f,i(i({ref:e},d),{},{components:t})):n.createElement(f,i({ref:e},d))}));function m(r,e){var t=arguments,o=e&&e.mdxType;if("string"==typeof r||o){var a=t.length,i=new Array(a);i[0]=u;var l={};for(var s in e)hasOwnProperty.call(e,s)&&(l[s]=e[s]);l.originalType=r,l.mdxType="string"==typeof r?r:o,i[1]=l;for(var p=2;p<a;p++)i[p]=t[p];return n.createElement.apply(null,i)}return n.createElement.apply(null,t)}u.displayName="MDXCreateElement"},8845:function(r,e,t){"use strict";t.r(e),t.d(e,{frontMatter:function(){return i},metadata:function(){return l},toc:function(){return s},default:function(){return d}});var n=t(2122),o=t(9756),a=(t(7294),t(3905)),i={sidebar_label:"errors",title:"fido.errors"},l={unversionedId:"reference/fido/errors",id:"reference/fido/errors",isDocsHomePage:!1,title:"fido.errors",description:"Error",source:"@site/docs/reference/fido/errors.md",sourceDirName:"reference/fido",slug:"/reference/fido/errors",permalink:"/fido/docs/reference/fido/errors",editUrl:"https://github.com/hojulian/fido/edit/documentation/docs/docs/reference/fido/errors.md",version:"current",sidebar_label:"errors",frontMatter:{sidebar_label:"errors",title:"fido.errors"},sidebar:"apiSidebar",previous:{title:"fido.core",permalink:"/fido/docs/reference/fido/core"},next:{title:"fido.time",permalink:"/fido/docs/reference/fido/time"}},s=[{value:"Error",id:"error",children:[{value:"__init__",id:"__init__",children:[]},{value:"msg",id:"msg",children:[]}]},{value:"RobotError",id:"roboterror",children:[]},{value:"SimulationError",id:"simulationerror",children:[]},{value:"SimulatorError",id:"simulatorerror",children:[]},{value:"WorldError",id:"worlderror",children:[]},{value:"DockerError",id:"dockererror",children:[]},{value:"NotImplementedError",id:"notimplementederror",children:[]},{value:"DTypeError",id:"dtypeerror",children:[]}],p={toc:s};function d(r){var e=r.components,t=(0,o.Z)(r,["components"]);return(0,a.kt)("wrapper",(0,n.Z)({},p,t,{components:e,mdxType:"MDXLayout"}),(0,a.kt)("h2",{id:"error"},"Error"),(0,a.kt)("pre",null,(0,a.kt)("code",{parentName:"pre",className:"language-python"},"class Error(Exception)\n")),(0,a.kt)("p",null,"A generic error that is raised when Fido execution fails."),(0,a.kt)("p",null,"Whenever possible, the session will raise a more specific subclass of\n",(0,a.kt)("inlineCode",{parentName:"p"},"Error")," from the ",(0,a.kt)("inlineCode",{parentName:"p"},"fido.errors")," module."),(0,a.kt)("h3",{id:"__init__"},"_","_","init","_","_"),(0,a.kt)("pre",null,(0,a.kt)("code",{parentName:"pre",className:"language-python"}," | def __init__(msg: str)\n")),(0,a.kt)("p",null,"Creates a new ",(0,a.kt)("inlineCode",{parentName:"p"},"Error")," indicating that an error has occurred."),(0,a.kt)("p",null,(0,a.kt)("strong",{parentName:"p"},"Arguments"),":"),(0,a.kt)("ul",null,(0,a.kt)("li",{parentName:"ul"},(0,a.kt)("inlineCode",{parentName:"li"},"msg")," ",(0,a.kt)("em",{parentName:"li"},"str")," - The message string describing the error.")),(0,a.kt)("h3",{id:"msg"},"msg"),(0,a.kt)("pre",null,(0,a.kt)("code",{parentName:"pre",className:"language-python"}," | @property\n | def msg() -> str\n")),(0,a.kt)("p",null,"The error message that describes the error."),(0,a.kt)("h2",{id:"roboterror"},"RobotError"),(0,a.kt)("pre",null,(0,a.kt)("code",{parentName:"pre",className:"language-python"},"class RobotError(Error)\n")),(0,a.kt)("p",null,"Represents a robot error raised by the underlying robot."),(0,a.kt)("p",null,"This is a wrapper around any error raised by ",(0,a.kt)("inlineCode",{parentName:"p"},"fido.robot.Robot"),"."),(0,a.kt)("h2",{id:"simulationerror"},"SimulationError"),(0,a.kt)("pre",null,(0,a.kt)("code",{parentName:"pre",className:"language-python"},"class SimulationError(Error)\n")),(0,a.kt)("p",null,"Represents a simulation error by the underlying simulation framework."),(0,a.kt)("p",null,"This is a wrapper around any error raised by ",(0,a.kt)("inlineCode",{parentName:"p"},"fido.simulation.Simulation"),"."),(0,a.kt)("h2",{id:"simulatorerror"},"SimulatorError"),(0,a.kt)("pre",null,(0,a.kt)("code",{parentName:"pre",className:"language-python"},"class SimulatorError(Error)\n")),(0,a.kt)("p",null,"Represents a simulator error by the underlying simulator."),(0,a.kt)("p",null,"This is a wrapper around any error raised by ",(0,a.kt)("inlineCode",{parentName:"p"},"fido.simulation.Simulator"),"."),(0,a.kt)("h2",{id:"worlderror"},"WorldError"),(0,a.kt)("pre",null,(0,a.kt)("code",{parentName:"pre",className:"language-python"},"class WorldError(Error)\n")),(0,a.kt)("p",null,"Represents a world error raised by the underlying world."),(0,a.kt)("p",null,"This is a wrapper around any error raised by ",(0,a.kt)("inlineCode",{parentName:"p"},"fido.world.World"),"."),(0,a.kt)("h2",{id:"dockererror"},"DockerError"),(0,a.kt)("pre",null,(0,a.kt)("code",{parentName:"pre",className:"language-python"},"class DockerError(Error)\n")),(0,a.kt)("p",null,"Represents a docker error raised by the underlying docker client."),(0,a.kt)("p",null,"This is a wrapper around any error raised by the internal docker client."),(0,a.kt)("h2",{id:"notimplementederror"},"NotImplementedError"),(0,a.kt)("pre",null,(0,a.kt)("code",{parentName:"pre",className:"language-python"},"class NotImplementedError(Error)\n")),(0,a.kt)("p",null,"Represents an error for calling a not yet implemented method."),(0,a.kt)("h2",{id:"dtypeerror"},"DTypeError"),(0,a.kt)("pre",null,(0,a.kt)("code",{parentName:"pre",className:"language-python"},"class DTypeError(Error)\n")),(0,a.kt)("p",null,"Represents an error for dType."))}d.isMDXComponent=!0}}]);