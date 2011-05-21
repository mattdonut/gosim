package main

import (
	"fmt"
	"vector"
	"poly"
	"solver"
	"os"
	"rand"
	"io"
	"strings"
	"net"
)
//Sim is a single simulation
type Sim struct {
	System []*poly.Chain //an array slice of chains, which is our system
	solver *solver.RK4 //the solver of choice
	solverInit bool //flag for checking the init state of the solver
}
func NewSim() *Sim{
	sim := new(Sim)
	sim.Init()
	return sim
}

//set up the basic data structures
func (s *Sim) Init() {
	s.System = make([]*poly.Chain,0)
	s.solver = new(solver.RK4)
	s.solver.Mods = make([]solver.Modifier,0)
	s.solverInit = false
}
//initialize the internal solver, as well as the Modifiers
func (s *Sim) InitSolver() {
	s.solver.Init(s.System)
	for _, mod := range s.solver.Mods {
		mod.Init(s.System)
	}
	s.solverInit = true
}

func (s *Sim) AddPoly(length int, origin *vector.D3) {
	//don't add a new chain if the solver has already been initialized
	if s.solverInit {
		return
	}
	fmt.Println("adding poly")
	chain := poly.New(length)
	//randomize the chain's coordinates, starting from origin
	chain.RandomLoc(origin)
	s.System = append(s.System, chain)
}
func (s *Sim) AddMod(mod solver.Modifier) {
	s.solver.Mods = append(s.solver.Mods, mod)
}
//write out the location state of the internal system as a JSON string
func (s *Sim) WriteLoc(output io.Writer) {
	locstrings := make([]string, len(s.System))
	for n , poly := range s.System{
		locstrings[n] = poly.ToJSON()
	}
	loc := strings.Join(locstrings, ",")
	io.WriteString(output,"[")
	io.WriteString(output,loc)
	io.WriteString(output,"]")
}

//RemoteSim is a network socket based controller for a Sim *NOT USED YET*
type RemoteSim struct {
	mySim *Sim
	conn net.Conn
	commandBuffer []byte
}
func NewRemoteSim()*RemoteSim{
	rs := new(RemoteSim)
	rs.mySim = NewSim()
	return rs
}
func (r *RemoteSim) dispatcher() bool {
	for {
		//read a command string off the connection
		nr, err := r.conn.Read(r.commandBuffer)
		if err!=nil {
			return false
		}
		commandString := string(r.commandBuffer[0:nr])
		
		//parse the command string
		parts := strings.Split(commandString,":",-1)
		cmd := parts[0]
		args := make([]string,0)
		if len(parts)>1 {
			args = parts[1:len(parts)]
		}
		
		//switch on the command,
		switch cmd {
		case "step":
			if len(args)<1 {
				return false
			}
		}
		//NOT DONE YET
		return true
	}
	return true
}

func main() {
	//start by setting the random seed
	rand.Seed(3141)
	
	//make a Sim
	mySim := NewSim()
	
	//add some polymers
	mySim.AddPoly(12,vector.NewD3(0,0,0))
	mySim.AddPoly(12,vector.NewD3(0,1,0))

	//add the basic polymer structer modifiers
	mySim.AddMod(solver.NewSpringForce(100))
	mySim.AddMod(solver.NewStiffForce(10))
	
	//pin all the polymers to their initial positions
	mySim.AddMod(solver.NewAutoPinForce(100))
	
	//add the Oseen Tensor modifier, note that the order is important here
	mySim.AddMod(solver.NewOseenTensor(.1))
	
	//add the Kinesin force modifier, AFTER the Oseen Tensor, to ensure that this force is not effected by it
	mySim.AddMod(solver.NewKinesinForce(1))

	//now that the polys and mods are in place, initialize the solver
	mySim.InitSolver()

	//set up the output stream, in this case a file
	file, err := os.Create("polyout.txt")
	if err!=nil {
		fmt.Println("file failure")
		return
	}
	
	
	fmt.Println("starting sim...")
	
	//write the opening container bracket for the JSON object
	file.WriteString("[")
	
	//run for a while
	for i := 0; i<20; i++ {
		fmt.Println(i)
		
		//take a sim step (note this may actually be several steps internally)
		mySim.solver.Step()
		//write out the location coordinates as a JSON string
		mySim.WriteLoc(file)
		//seperate the individial location arrays with a comma
		file.WriteString(",")
	}
	//remove the last comma
	file.Seek(-1,1)
	//close the container bracket
	file.WriteString("]")
	
	//done
	fmt.Println("sim complete")
}
