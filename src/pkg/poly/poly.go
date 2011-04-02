package poly

import (
	"vector"
)

type Chain struct {
	length int
	Loc, Vel []*vector.D3
}

func New(length int) *Chain {
	chain := new(Chain)
	chain.length = length
	chain.Loc = make([]*vector.D3,length)
	chain.Vel = make([]*vector.D3,length)
	for index ,_ := range chain.Loc {
		chain.Loc[index] = vector.NewD3(0,0,0)
		chain.Vel[index] = vector.NewD3(0,0,0)
	}
	return chain
}

func NewRandom(length int) *Chain {
	chain := new(Chain)
	chain.length = length
	chain.Loc = make([]*vector.D3,length)
	chain.Vel = make([]*vector.D3,length)
	last := vector.NewD3(0,0,0)
	for index ,_ := range chain.Loc {
		chain.Loc[index] = last.Add(vector.RandomD3().Normalize())
		last = chain.Loc[index]
		chain.Vel[index] = vector.RandomD3().Normalize()
	}
	return chain
}

func (c *Chain) RandomLoc(origin *vector.D3) *Chain {
	for index ,_ := range c.Loc {
		c.Loc[index] = origin.Add(vector.RandomD3().Normalize())
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

func (c *Chain) GetLength() int {
	return c.length
}

func (c *Chain) ToJSON() string {
	output := "["
	for _, r := range c.Loc {
		output += "[" + r.String() + "]" +","
	}
	output = output[0:len(output)-1]
	output+= "]"
	return output
}