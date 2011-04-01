package poly

import (
	"vector"
	"fmt"
)

type Chain struct {
	length int
	Loc, Vel, Acc []*vector.D3
}

func New(length int) *Chain {
	chain := new(Chain)
	chain.length = length
	chain.Loc = make([]*vector.D3,length)
	chain.Vel = make([]*vector.D3,length)
	chain.Acc = make([]*vector.D3,length)
	for index ,_ := range chain.Loc {
		chain.Loc[index] = vector.NewD3(0,0,0)
		chain.Vel[index] = vector.NewD3(0,0,0)
		chain.Acc[index] = vector.NewD3(0,0,0)
	}
	return chain
}

func NewRandom(length int) *Chain {
	chain := new(Chain)
	chain.length = length
	chain.Loc = make([]*vector.D3,length)
	chain.Vel = make([]*vector.D3,length)
	chain.Acc = make([]*vector.D3,length)
	last := vector.NewD3(0,0,0)
	for index ,_ := range chain.Loc {
		chain.Loc[index] = last.Add(vector.RandomD3().Normalize())
		last = chain.Loc[index]
		chain.Vel[index] = vector.RandomD3().Normalize()
		chain.Acc[index] = vector.RandomD3().Normalize()
	}
	return chain
}

func (c *Chain) RandomLoc(origin *vector.D3) *Chain {
	for index ,_ := range c.Loc {
		c.Loc[index] = origin.Add(vector.RandomD3().Normalize().Mul(1.1))
		origin = c.Loc[index]
	}
	return c
}

func (c *Chain) RandomVel() *Chain {
	for index ,_ := range c.Vel {
		c.Vel[index] = vector.RandomD3().Normalize()
	}
	return c
}

func (c *Chain) RandomAcc() *Chain {
	for index ,_ := range c.Acc {
		c.Acc[index] = vector.RandomD3().Normalize()
	}
	return c
}

func (c *Chain) GetLength() int {
	return c.length
}

func (c *Chain) Print() {
	fmt.Printf("Locitions:\n")
	for _ , value := range c.Loc {
		fmt.Printf(value.String()+"\n")
	}
	fmt.Printf("Velocities:\n")
	for _ , value := range c.Vel {
		fmt.Printf(value.String()+"\n")
	}
//	fmt.Printf("Forces:\n")
//	for _ , value := range c.Acc {
//		fmt.Printf(value.String()+"\n")
//	}
}