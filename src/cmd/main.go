package main

import (
	"fmt"
	"vector"
	"poly"
	"solver"
	"os"
	"rand"
)

func main() {
	fmt.Printf("Hello, world, again, and again!\n")
	system := make([]*poly.Chain,0)
	blank := poly.New(64)
	other := poly.New(64)
	third := poly.New(64)
	rand.Seed(3141)
	blank.RandomLoc(vector.NewD3(0,0,0))
	other.RandomLoc(vector.NewD3(0,-3,0))
	third.RandomLoc(vector.NewD3(3.5,0,0))
	system = append(system, blank)
	system = append(system, other)
	system = append(system, third)
	//fmt.Println(blank.ToJSON())
	sim := new(solver.RK4)
	sim.Init(system)
	
	sim.Mods = make([]solver.Modifier, 1)
	sim.Mods[0] = solver.NewSpringForce(100)
	sim.Mods = append(sim.Mods, solver.NewStiffForce(10))

	sim.Mods = append(sim.Mods, solver.NewPinForce(100, 0, 0, vector.NewD3(0,0,0)))
	sim.Mods = append(sim.Mods, solver.NewPinForce(100, 1, 0, vector.NewD3(0,-3,0)))
	sim.Mods = append(sim.Mods, solver.NewPinForce(100, 2, 0, vector.NewD3(3.5,0,0)))
	//sim.Mods = append(sim.Mods, solver.NewPinForce(100, 0, 1, vector.NewD3(0,0,1)))
	//sim.Mods = append(sim.Mods, solver.NewPinForce(100, 1, 1, vector.NewD3(0,3,1)))
	//sim.Mods = append(sim.Mods, solver.NewPinForce(100, 2, 1, vector.NewD3(3.5,0,1)))
	//rk.Mods = append(rk.Mods, NewConstForce(vector.NewD3(0,0,1)))
	sim.Mods = append(sim.Mods, solver.NewOseenTensor(.1,sim.GetSystem()[0]))
	sim.Mods = append(sim.Mods, solver.NewKinesinForce(1))
	
	
	file, err := os.Open("polyout.txt", os.O_WRONLY | os.O_CREATE ,0666 )
	if err != nil {
		return
	}
	defer file.Close()
	file.WriteString("[")
	file.WriteString("[")
	for _, poly := range system{
		file.WriteString(poly.ToJSON())
		file.WriteString(",")
	}
	file.Seek(-1,1)
	file.WriteString("]")
	file.WriteString(",")
	for i := 0; i < 600 ; i++ {
		sim.Step()
		if i % 2 == 1{
			//fmt.Println(blank.ToJSON())
			fmt.Println(i)
			file.WriteString("[")
			for _, poly := range system{
				file.WriteString(poly.ToJSON())
				file.WriteString(",")
			}
			file.Seek(-1,1)
			file.WriteString("]")
			file.WriteString(",")
		}
	}
	file.Seek(-1,1)
	file.WriteString("]")
	fmt.Printf("End of the line")
}
