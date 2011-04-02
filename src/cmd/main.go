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

	blank := poly.New(64)
	blank.RandomLoc(vector.NewD3(0,0,0))

	fmt.Println(blank.ToJSON())
	solver := new(solver.RK4)
	solver.Init(blank)
	
	file, err := os.Open("polyout.txt", os.O_WRONLY | os.O_CREATE ,0666 )
	if err != nil {
		return
	}
	defer file.Close()
	file.WriteString("[")
	file.WriteString(blank.ToJSON())
	file.WriteString(",")
	for i := 0; i < 300 ; i++ {
		solver.Step()
		if i % 3 == 1{
			fmt.Println(blank.ToJSON())
			file.WriteString(blank.ToJSON())
			file.WriteString(",")
		}
	}
	file.Seek(-1,1)
	file.WriteString("]")
	fmt.Printf("End of the line")
}
