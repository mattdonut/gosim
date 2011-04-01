package solver

import (
	"poly"
	"vector"
)

type Solver interface {
	Init(chain *poly.Chain)
	Step() *poly.Chain
}

type Modifier interface {
	Act(input, output *poly.Chain)
}

type Newton struct {
	h float64
	Sys *poly.Chain
	Mod Modifier
}

func (newt *Newton) Init(chain *poly.Chain) {
	newt.Sys = chain
	newt.Mod = NewSpringForce(10)
	newt.h = .001
}

func (newt *Newton) Step() *poly.Chain {
	newt.Mod.Act(newt.Sys, newt.Sys)
	for i, _ := range newt.Sys.Loc {
		newt.Sys.Loc[i] = newt.Sys.Loc[i].Add(newt.Sys.Vel[i].Mul(newt.h))
	}
	return newt.Sys
}

type RK4 struct {
	h,h2,h6 float64
	Sys, sys2,sys3,sys4 *poly.Chain
	Mods []Modifier
}

func (rk *RK4) Init(chain *poly.Chain) {
	rk.Sys = chain
	rk.sys2 = poly.New(chain.GetLength())
	rk.sys3 = poly.New(chain.GetLength())
	rk.sys4 = poly.New(chain.GetLength())
	rk.Mods = make([]Modifier, 1)
	rk.Mods[0] = NewSpringForce(10)
	rk.h = .01
	rk.h2 = rk.h/2
	rk.h6 = rk.h/6
}

func (rk *RK4) Step() *poly.Chain {
	//take a step using RK4
	for n := 0; n < 100 ; n ++ {
	//clear the forces
	for i, _ := range rk.Sys.Vel {
		rk.Sys.Vel[i] = vector.NewD3(0,0,0)
		rk.sys2.Vel[i] = vector.NewD3(0,0,0)
		rk.sys3.Vel[i] = vector.NewD3(0,0,0)
		rk.sys4.Vel[i] = vector.NewD3(0,0,0)
	}
	//calculate forces at the initial point (k1)
	for _, mod := range rk.Mods {
		mod.Act(rk.Sys, rk.Sys)
	}
	//determine the locations for the first midpoint step
	for i, _ := range rk.Sys.Loc {
		rk.sys2.Loc[i] = rk.Sys.Loc[i].Add(rk.Sys.Vel[i].Mul(rk.h2))
	}
	//calculate forces for the first midpoint (k2)
	for _, mod := range rk.Mods {
		mod.Act(rk.sys2, rk.sys2)
	}
	//determine the locations foe the second midpoint step
	for i, _ := range rk.Sys.Loc {
		rk.sys3.Loc[i] = rk.Sys.Loc[i].Add(rk.sys2.Vel[i].Mul(rk.h2))
	}
	//calculate forces for the second midpoint (k3)
	for _, mod := range rk.Mods {
		mod.Act(rk.sys3, rk.sys3)
	}
	//determine the locations foe the second midpoint step
	for i, _ := range rk.Sys.Loc {
		rk.sys4.Loc[i] = rk.Sys.Loc[i].Add(rk.sys3.Vel[i].Mul(rk.h))
	}
	//calculate forces for the endpoint
	for _, mod := range rk.Mods {
		mod.Act(rk.sys4, rk.sys4)
	}
	//apply the transform to the primary system
	for i, _ := range rk.Sys.Loc {
		rk.Sys.Loc[i] = rk.Sys.Loc[i].Add(rk.Sys.Vel[i].Add( rk.sys2.Vel[i].Add(rk.sys3.Vel[i]).Mul(2) ).Add( rk.sys4.Vel[i] ).Mul(rk.h6) )
	}

	}
	return rk.Sys
}

type SpringForce struct {
	k float64
}
func NewSpringForce(k float64) Modifier {
	sf := new(SpringForce)
	sf.k = k
	return sf
}
func (mod *SpringForce) Act(in, out *poly.Chain) {
	var f1, f2 *vector.D3
	for i, r := range in.Loc[1:len(in.Loc)-1] {
		f1 = in.Loc[i].Sub(r)
		f2 = r.Sub(in.Loc[i+2])
		//out.Vel[i] = vector.NewD3(2,2,2)
		out.Vel[i+1] = f1.Sub(f1.Normalize()).Sub(f2.Sub(f2.Normalize())).Mul(mod.k)
	}
	f1 = in.Loc[1].Sub(in.Loc[0])
	f2 = in.Loc[len(in.Loc)-2].Sub(in.Loc[len(in.Loc)-1])
	out.Vel[0] = f1.Sub(f1.Normalize()).Mul(mod.k)
	out.Vel[len(in.Vel)-1] = f2.Sub(f2.Normalize()).Mul(mod.k)
}