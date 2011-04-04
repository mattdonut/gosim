package main

import (
	"fmt"
	"vector"
	"poly"
	"solver"
	"os"
)

func main() {
	fmt.Printf("Hello, world, again, and again!\n")
	system := make([]*poly.Chain,1)
	blank := poly.New(32)
	other := poly.New(32)
	blank.RandomLoc(vector.NewD3(0,0,0))
	other.RandomLoc(vector.NewD3(1,1,0))
	system[0] = blank
	system = append(system, other)
	fmt.Println(blank.ToJSON())
	solver := new(solver.RK4)
	solver.Init(system)
	
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
	for i := 0; i < 1000 ; i++ {
		solver.Step()
		if i % 3 == 1{
			fmt.Println(blank.ToJSON())
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
