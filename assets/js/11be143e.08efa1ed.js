(self.webpackChunkdocs=self.webpackChunkdocs||[]).push([[9135],{71625:function(t,e,n){"use strict";n.r(e),n.d(e,{default:function(){return h},frontMatter:function(){return p},metadata:function(){return c},toc:function(){return u}});var o=n(22122),a=n(19756),r=n(67294),i=n(3905),s=n(21140),l=n.n(s);l().initialize({startOnLoad:!0});var d=function(t){var e=t.chart;return(0,r.useEffect)((function(){l().contentLoaded()}),[]),r.createElement("div",{className:"mermaid"}," ",e," ")},p={},c={unversionedId:"concepts",id:"concepts",isDocsHomePage:!1,title:"Concepts",description:"Overview",source:"@site/docs/concepts.mdx",sourceDirName:".",slug:"/concepts",permalink:"/fido/docs/concepts",editUrl:"https://github.com/hojulian/fido/edit/documentation/docs/docs/concepts.mdx",version:"current",frontMatter:{},sidebar:"mainSidebar",previous:{title:"Examples",permalink:"/fido/docs/examples"},next:{title:"Advanced Features",permalink:"/fido/docs/advanced"}},u=[{value:"Overview",id:"overview",children:[]},{value:"Simulation",id:"simulation",children:[]},{value:"Simulator",id:"simulator",children:[{value:"Gazebo",id:"gazebo",children:[]}]},{value:"World",id:"world",children:[]},{value:"Robot",id:"robot",children:[{value:"Sensor",id:"sensor",children:[]}]},{value:"Docker",id:"docker",children:[]}],m={toc:u};function h(t){var e=t.components,n=(0,a.Z)(t,["components"]);return(0,i.kt)("wrapper",(0,o.Z)({},m,n,{components:e,mdxType:"MDXLayout"}),(0,i.kt)("h2",{id:"overview"},"Overview"),(0,i.kt)("h2",{id:"simulation"},"Simulation"),(0,i.kt)(d,{chart:"\n\tgraph TD;\n\t\tsim(Simulation);\n\t\tsimulator(Simulator);\n\t\trobot(Robot);\n\t\tworld(World);\n\t\tsim --\x3e robot;\n\t\tsim --\x3e simulator;\n\t\tsim --\x3e world;\n",mdxType:"Mermaid"}),(0,i.kt)("p",null,"A simulation represents a simulated environment. A ",(0,i.kt)("a",{parentName:"p",href:"#world"},"World"),", ",(0,i.kt)("a",{parentName:"p",href:"#robot"},"Robot"),", and ",(0,i.kt)("a",{parentName:"p",href:"#simulator"},"Simulator")," jointly forms a Simulation."),(0,i.kt)("p",null,"To create a simulation, simply:"),(0,i.kt)("pre",null,(0,i.kt)("code",{parentName:"pre",className:"language-python"},"sim = Simulation(simulator=..., world=...)\n")),(0,i.kt)("p",null,"Architecturally, a simulation is self-contained as a docker container. By default, a simulation uses the Fido base image. ",(0,i.kt)("em",{parentName:"p"},"Note: it is possible to use a different docker image, see ",(0,i.kt)("a",{parentName:"em",href:"./advanced#custom-docker-image"},"Custom Docker Image"),".")," Each simulation has an unique docker container."),(0,i.kt)("p",null,"Internally, when a simulation is created, it will first initialize by loading all the external world and robot files using ",(0,i.kt)("inlineCode",{parentName:"p"},"rosinstall"),". The loaded files are located under ",(0,i.kt)("inlineCode",{parentName:"p"},".fido/sim-$SIM_ID")," folder. After that, it will create a docker container with the files attached to its catkin workspace. The container created will be in a pause state."),(0,i.kt)("p",null,"When the container is started, it creates a ",(0,i.kt)("inlineCode",{parentName:"p"},"rosbridge")," and a ",(0,i.kt)("inlineCode",{parentName:"p"},"novnc")," GUI on two randomized ports. The simulation will then attempt to connect to the container using its ROS client. Once it is connected successfully, it will then start up all the sensors of each robots by subscribing to the corresponding topics. This allows the robots to have the latest state updates. The simulation is controlled using simulator's ",(0,i.kt)("a",{parentName:"p",href:"./reference/fido/simulation/simulator"},"API"),"."),(0,i.kt)("p",null,"At this stage, the simulation will continue to run in the background until ",(0,i.kt)("inlineCode",{parentName:"p"},"stop()")," or ",(0,i.kt)("inlineCode",{parentName:"p"},"destroy()")," is called. When ",(0,i.kt)("inlineCode",{parentName:"p"},"destroy()")," is called, the simulation is forcefully stopped and destroyed alongside with its underlying docker container. A destroyed simulation cannot be started again."),(0,i.kt)("h2",{id:"simulator"},"Simulator"),(0,i.kt)("p",null,"A simulator acts as the physics engine for running a simulation. This provides API for the simulation to control the lifecycle of a simulator (e.g. starting & stopping). Fido does not make any enforcement on the compatibility of simulator, world, and robot, it is up to the user to check their compatibility."),(0,i.kt)("p",null,"The basic operations of simulator are as follows,"),(0,i.kt)("ul",null,(0,i.kt)("li",{parentName:"ul"},"Start/Stop"),(0,i.kt)("li",{parentName:"ul"},"Reset"),(0,i.kt)("li",{parentName:"ul"},"View"),(0,i.kt)("li",{parentName:"ul"},"Show time")),(0,i.kt)("p",null,"It is up to the implementation to decide what each of these operation means."),(0,i.kt)("h3",{id:"gazebo"},"Gazebo"),(0,i.kt)("p",null,(0,i.kt)("a",{parentName:"p",href:"http://gazebosim.org/"},"Gazebo")," is a simulator specifically designed for robot simulation."),(0,i.kt)("p",null,"Currently, this is the only supported simulator."),(0,i.kt)("h2",{id:"world"},"World"),(0,i.kt)("p",null,"A world represents a collection of robots and objects. The common world operations are adding and removing robots."),(0,i.kt)("p",null,"Internally, the world is defined as a set of files: A ",(0,i.kt)("inlineCode",{parentName:"p"},".rosinstall")," file for importing external world models, and a ",(0,i.kt)("inlineCode",{parentName:"p"},".launch")," file for starting the world. The ",(0,i.kt)("inlineCode",{parentName:"p"},"export_files()")," function exports all the files needed to represent the world and all the included robots."),(0,i.kt)("h2",{id:"robot"},"Robot"),(0,i.kt)("p",null,"A robot represents either a physical or a simulated robot."),(0,i.kt)("p",null,"To control a physical robot, first, start a ",(0,i.kt)("a",{parentName:"p",href:"http://wiki.ros.org/rosbridge_server"},"rosbridge server")," on the robot, and expose a port for connection. Then, in python simply do,"),(0,i.kt)("pre",null,(0,i.kt)("code",{parentName:"pre",className:"language-python"},'robot = Turtlebot3("example-bot", physical=True)\nrobot.connect(host="127.0.0.1", port=9090) # Robot\'s IP address and rosbridge port\n')),(0,i.kt)("p",null,"To control a simulated robot, a simulated world is needed,"),(0,i.kt)("pre",null,(0,i.kt)("code",{parentName:"pre",className:"language-python"},'world = RaceTrack()\nrobot = Turtlebot3("example-bot", physical=False)\nworld.add(robot) # Add simulated robot to world\n')),(0,i.kt)("p",null,"A basic structure of a robot looks like this,"),(0,i.kt)(d,{chart:"\n\tgraph TD;\n\t\ts1(Odom Sensor);\n\t\ts2(Lidar Sensor);\n\t\tsubgraph functions;\n\t\t\tmove(Move);\n\t\t\trotate(Rotate);\n\t\tend;\n\t\tsubgraph sensors;\n\t\t\ts1;\n\t\t\ts2;\n\t\tend;\n\t\tsubgraph fields;\n\t\t\tx;\n\t\t\ty;\n\t\t\tz;\n\t\t\trange;\t\n\t\tend;\n\t\tsubgraph robot;\n\t\t\tfields;\n\t\t\tfunctions;\n\t\t\tsensors;\n\t\tend;\n",mdxType:"Mermaid"}),(0,i.kt)("h3",{id:"sensor"},"Sensor"),(0,i.kt)(d,{chart:"\n\tgraph TD;\n\t\tsensor(Sensor);\n\t\todom(Odomer);\n\t\tlidar(Lidar);\n\t\tcamera(Camera);\n\t\tsensor --\x3e odom;\n\t\tsensor --\x3e lidar;\n\t\tsensor --\x3e camera;\n",mdxType:"Mermaid"}),(0,i.kt)("p",null,"A sensor represents a sensor on the robot."),(0,i.kt)("p",null,"Internally, a sensor listens to a specific ROS topic and updates the robot's internal state. For instance, a ",(0,i.kt)("inlineCode",{parentName:"p"},"Odomer")," listens to the ",(0,i.kt)("inlineCode",{parentName:"p"},"/odom")," topic and update the robot's ",(0,i.kt)("inlineCode",{parentName:"p"},"x"),", ",(0,i.kt)("inlineCode",{parentName:"p"},"y"),", ",(0,i.kt)("inlineCode",{parentName:"p"},"z")," coordinates. A sensor only starts listening to updates once the simulation has started. It will stop updating once the simulation is destroyed."),(0,i.kt)(d,{chart:"\n\tgraph TD;\n\t\ts1(Odom Sensor);\n\t\ts2(Lidar Sensor);\n\t\tsubgraph functions;\n\t\t\tmove(Move);\n\t\t\trotate(Rotate);\n\t\tend;\n\t\tsubgraph sensors;\n\t\t\ts1;\n\t\t\ts2;\n\t\tend;\n\t\tsubgraph fields;\n\t\t\ts1 --\x3e|updates| x;\n\t\t\ts1 --\x3e|updates| y;\n\t\t\ts1 --\x3e|updates| z;\n\t\t\ts2 --\x3e|updates| range;\t\n\t\tend;\n\t\tsubgraph robot;\n\t\t\tfields;\n\t\t\tfunctions;\n\t\t\tsensors;\n\t\tend;\n",mdxType:"Mermaid"}),(0,i.kt)("h2",{id:"docker"},"Docker"),(0,i.kt)("p",null,"Fido leverages Docker for simulation. This allows each simulation to be isolated and platform independent."),(0,i.kt)("p",null,"To change the Docker host,"),(0,i.kt)("pre",null,(0,i.kt)("code",{parentName:"pre",className:"language-python"},"from fido import config\n\nconfig.set_docker_host(base_url=..., version=...)\n")))}h.isMDXComponent=!0},11748:function(t,e,n){var o={"./locale":89234,"./locale.js":89234};function a(t){var e=r(t);return n(e)}function r(t){if(!n.o(o,t)){var e=new Error("Cannot find module '"+t+"'");throw e.code="MODULE_NOT_FOUND",e}return o[t]}a.keys=function(){return Object.keys(o)},a.resolve=r,t.exports=a,a.id=11748}}]);