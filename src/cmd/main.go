package main

import (
	"fmt"
	"vector"
	"poly"
	"solver"
)


func main() {
	fmt.Printf("Hello, world, again, and again!\n")
	//chain := poly.NewRandom(8)
	blank := poly.New(4)
	//blank.RandomLoc(vector.NewD3(0,0,0))
	blank.Loc[0] = vector.NewD3(0,0,0)
	blank.Loc[1] = vector.NewD3(1,0,0)
	blank.Loc[2] = vector.NewD3(2,0,0)
	blank.Loc[3] = vector.NewD3(3,.1,0)
	blank.Print()
	solver := new(solver.RK4)
	solver.Init(blank)
	for i := 0; i < 100 ; i++ {
		solver.Step()
		if i % 10 == 1{
			blank.Print()
		}
	}
	fmt.Printf("End of the road")
}
