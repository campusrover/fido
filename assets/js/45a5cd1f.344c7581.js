(self.webpackChunkdocs=self.webpackChunkdocs||[]).push([[3365],{3905:function(e,t,n){"use strict";n.d(t,{Zo:function(){return d},kt:function(){return m}});var o=n(7294);function r(e,t,n){return t in e?Object.defineProperty(e,t,{value:n,enumerable:!0,configurable:!0,writable:!0}):e[t]=n,e}function i(e,t){var n=Object.keys(e);if(Object.getOwnPropertySymbols){var o=Object.getOwnPropertySymbols(e);t&&(o=o.filter((function(t){return Object.getOwnPropertyDescriptor(e,t).enumerable}))),n.push.apply(n,o)}return n}function a(e){for(var t=1;t<arguments.length;t++){var n=null!=arguments[t]?arguments[t]:{};t%2?i(Object(n),!0).forEach((function(t){r(e,t,n[t])})):Object.getOwnPropertyDescriptors?Object.defineProperties(e,Object.getOwnPropertyDescriptors(n)):i(Object(n)).forEach((function(t){Object.defineProperty(e,t,Object.getOwnPropertyDescriptor(n,t))}))}return e}function l(e,t){if(null==e)return{};var n,o,r=function(e,t){if(null==e)return{};var n,o,r={},i=Object.keys(e);for(o=0;o<i.length;o++)n=i[o],t.indexOf(n)>=0||(r[n]=e[n]);return r}(e,t);if(Object.getOwnPropertySymbols){var i=Object.getOwnPropertySymbols(e);for(o=0;o<i.length;o++)n=i[o],t.indexOf(n)>=0||Object.prototype.propertyIsEnumerable.call(e,n)&&(r[n]=e[n])}return r}var s=o.createContext({}),c=function(e){var t=o.useContext(s),n=t;return e&&(n="function"==typeof e?e(t):a(a({},t),e)),n},d=function(e){var t=c(e.components);return o.createElement(s.Provider,{value:t},e.children)},p={inlineCode:"code",wrapper:function(e){var t=e.children;return o.createElement(o.Fragment,{},t)}},u=o.forwardRef((function(e,t){var n=e.components,r=e.mdxType,i=e.originalType,s=e.parentName,d=l(e,["components","mdxType","originalType","parentName"]),u=c(n),m=r,f=u["".concat(s,".").concat(m)]||u[m]||p[m]||i;return n?o.createElement(f,a(a({ref:t},d),{},{components:n})):o.createElement(f,a({ref:t},d))}));function m(e,t){var n=arguments,r=t&&t.mdxType;if("string"==typeof e||r){var i=n.length,a=new Array(i);a[0]=u;var l={};for(var s in t)hasOwnProperty.call(t,s)&&(l[s]=t[s]);l.originalType=e,l.mdxType="string"==typeof e?e:r,a[1]=l;for(var c=2;c<i;c++)a[c]=n[c];return o.createElement.apply(null,a)}return o.createElement.apply(null,n)}u.displayName="MDXCreateElement"},8923:function(e,t,n){"use strict";n.r(t),n.d(t,{frontMatter:function(){return a},metadata:function(){return l},toc:function(){return s},default:function(){return d}});var o=n(2122),r=n(9756),i=(n(7294),n(3905)),a={},l={unversionedId:"concepts",id:"concepts",isDocsHomePage:!1,title:"Concepts",description:"Simulation",source:"@site/docs/concepts.md",sourceDirName:".",slug:"/concepts",permalink:"/fido/docs/concepts",editUrl:"https://github.com/hojulian/fido/edit/documentation/docs/docs/concepts.md",version:"current",frontMatter:{},sidebar:"mainSidebar",previous:{title:"Examples",permalink:"/fido/docs/examples"},next:{title:"Advanced Features",permalink:"/fido/docs/advanced"}},s=[{value:"Simulation",id:"simulation",children:[]},{value:"Simulator",id:"simulator",children:[{value:"Gazebo",id:"gazebo",children:[]}]},{value:"World",id:"world",children:[]},{value:"Robot",id:"robot",children:[{value:"Sensor",id:"sensor",children:[]}]},{value:"Docker",id:"docker",children:[]}],c={toc:s};function d(e){var t=e.components,n=(0,r.Z)(e,["components"]);return(0,i.kt)("wrapper",(0,o.Z)({},c,n,{components:t,mdxType:"MDXLayout"}),(0,i.kt)("h2",{id:"simulation"},"Simulation"),(0,i.kt)("p",null,"A simulation represents a simulated environment. A ",(0,i.kt)("a",{parentName:"p",href:"#world"},"World"),", ",(0,i.kt)("a",{parentName:"p",href:"#robot"},"Robot"),", and ",(0,i.kt)("a",{parentName:"p",href:"#simulator"},"Simulator")," jointly forms a Simulation."),(0,i.kt)("p",null,"To create a simulation, simply:"),(0,i.kt)("pre",null,(0,i.kt)("code",{parentName:"pre",className:"language-python"},"sim = Simulation(simulator=..., world=...)\n")),(0,i.kt)("p",null,"Architecturally, a simulation is self-contained as a docker container. By default, a simulation uses the Fido base image. ",(0,i.kt)("em",{parentName:"p"},"Note: it is possible to use a different docker image, see ",(0,i.kt)("a",{parentName:"em",href:"./advanced#custom-docker-image"},"Custom Docker Image"),".")," Each simulation has an unique docker container."),(0,i.kt)("p",null,"Internally, when a simulation is created, it will first initialize by loading all the external world and robot files using ",(0,i.kt)("inlineCode",{parentName:"p"},"rosinstall"),". The loaded files are located under ",(0,i.kt)("inlineCode",{parentName:"p"},".fido/sim-$SIM_ID")," folder. After that, it will create a docker container with the files attached to its catkin workspace. The container created will be in a pause state."),(0,i.kt)("p",null,"When the container is started, it creates a ",(0,i.kt)("inlineCode",{parentName:"p"},"rosbridge")," and a ",(0,i.kt)("inlineCode",{parentName:"p"},"novnc")," GUI on two randomized ports. The simulation will then attempt to connect to the container using its ROS client. Once it is connected successfully, it will then start up all the sensors of each robots by subscribing to the corresponding topics. This allows the robots to have the latest state updates. The simulation is controlled using simulator's ",(0,i.kt)("a",{parentName:"p",href:"./reference/fido/simulation/simulator"},"API"),"."),(0,i.kt)("p",null,"At this stage, the simulation will continue to run in the background until ",(0,i.kt)("inlineCode",{parentName:"p"},"stop()")," or ",(0,i.kt)("inlineCode",{parentName:"p"},"destroy()")," is called. When ",(0,i.kt)("inlineCode",{parentName:"p"},"destroy()")," is called, the simulation is forcefully stopped and destroyed alongside with its underlying docker container. A destroyed simulation cannot be started again."),(0,i.kt)("h2",{id:"simulator"},"Simulator"),(0,i.kt)("p",null,"A simulator acts as the physics engine for running a simulation. This provides API for the simulation to control the lifecycle of a simulator (e.g. starting & stopping). Fido does not make any enforcement on the compatibility of simulator, world, and robot, it is up to the user to check their compatibility."),(0,i.kt)("p",null,"The basic operations of simulator are as follows,"),(0,i.kt)("ul",null,(0,i.kt)("li",{parentName:"ul"},"Start/Stop"),(0,i.kt)("li",{parentName:"ul"},"Reset"),(0,i.kt)("li",{parentName:"ul"},"View"),(0,i.kt)("li",{parentName:"ul"},"Show time")),(0,i.kt)("p",null,"It is up to the implementation to decide what each of these operation means."),(0,i.kt)("h3",{id:"gazebo"},"Gazebo"),(0,i.kt)("p",null,(0,i.kt)("a",{parentName:"p",href:"http://gazebosim.org/"},"Gazebo")," is a simulator specifically designed for robot simulation."),(0,i.kt)("p",null,"Currently, this is the only supported simulator."),(0,i.kt)("h2",{id:"world"},"World"),(0,i.kt)("p",null,"A world represents a collection of robots and objects. The common world operations are adding and removing robots."),(0,i.kt)("p",null,"Internally, the world is defined as a set of files: A ",(0,i.kt)("inlineCode",{parentName:"p"},".rosinstall")," file for importing external world models, and a ",(0,i.kt)("inlineCode",{parentName:"p"},".launch")," file for starting the world. The ",(0,i.kt)("inlineCode",{parentName:"p"},"export_files()")," function exports all the files needed to represent the world and all the included robots."),(0,i.kt)("h2",{id:"robot"},"Robot"),(0,i.kt)("p",null,"A robot represents either a physical or a simulated robot."),(0,i.kt)("h3",{id:"sensor"},"Sensor"),(0,i.kt)("p",null,"A sensor represents a sensor on the robot."),(0,i.kt)("p",null,"Internally, a sensor listens to a specific ROS topic and updates the robot's internal state. For instance, a ",(0,i.kt)("inlineCode",{parentName:"p"},"Odomer")," listens to the ",(0,i.kt)("inlineCode",{parentName:"p"},"/odom")," topic and update the robot's ",(0,i.kt)("inlineCode",{parentName:"p"},"x"),", ",(0,i.kt)("inlineCode",{parentName:"p"},"y"),", ",(0,i.kt)("inlineCode",{parentName:"p"},"z")," coordinates. A sensor only starts listening to updates once the simulation has started. It will stop updating once the simulation is destroyed."),(0,i.kt)("h2",{id:"docker"},"Docker"),(0,i.kt)("p",null,"Fido leverages Docker for simulation. This allows each simulation to be isolated and platform independent."))}d.isMDXComponent=!0}}]);