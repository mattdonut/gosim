package poly

import (
	"vector"
)
//a simple representation of a polymer of a fixed length
type Chain struct {
	length int
	Loc, Vel []*vector.D3 //two vector array slices for the locations and velocities of the monomers
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
//make a new vector with random positions and velocities
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
//replace the locations of the chain with a random walk starting from origin
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
//mirror the x coordinates of the entire chain
func (c *Chain) MirrorX() *Chain {
	for index ,_ := range c.Loc {
		c.Loc[index].MirrorX()
	}
	return c
}
//mirror the y coordinates of the entire chain
func (c *Chain) MirrorY() *Chain {
	for _, v := range c.Loc {
		v.MirrorY()
	}
	return c
}
func (c *Chain) GetLength() int {
	return c.length
}
//return a JSON string representation of the chain's Location coordinates
func (c *Chain) ToJSON() string {
	output := "["
	for _, r := range c.Loc {
		output += "[" + r.String() + "]" +","
	}
	output = output[0:len(output)-1]
	output+= "]"
	return output
}