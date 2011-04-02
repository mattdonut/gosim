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
	rk.Mods[0] = NewSpringForce(100)
	rk.Mods = append(rk.Mods, NewStiffForce(10))
	rk.Mods = append(rk.Mods, NewKinesinForce(1))
	rk.Mods = append(rk.Mods, NewPinForce(100, 0, vector.NewD3(0,0,0)))
	rk.Mods = append(rk.Mods, NewConstForce(vector.NewD3(0,0,1)))
	rk.h = .003
	rk.h2 = rk.h/2
	rk.h6 = rk.h/6
}

func (rk *RK4) Step() *poly.Chain {
	//take a step using RK4
	for n := 0; n < 100 ; n ++ {
	//clear the forces
	for i, _ := range rk.Sys.Vel {
		rk.Sys.Vel[i].Zero()
		rk.sys2.Vel[i].Zero()
		rk.sys3.Vel[i].Zero()
		rk.sys4.Vel[i].Zero()
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

	}//end multiplicity loop
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
		out.Vel[i+1] = out.Vel[i+1].Add(f1.Sub(f1.Normalize()).Sub(f2.Sub(f2.Normalize())).Mul(mod.k))
	}
	f1 = in.Loc[1].Sub(in.Loc[0])
	f2 = in.Loc[len(in.Loc)-2].Sub(in.Loc[len(in.Loc)-1])
	out.Vel[0] = out.Vel[0].Add(f1.Sub(f1.Normalize()).Mul(mod.k))
	out.Vel[len(in.Vel)-1] = out.Vel[len(in.Vel)-1].Add(f2.Sub(f2.Normalize()).Mul(mod.k))
}
type StiffForce struct {
	k float64
}
func NewStiffForce(k float64) Modifier{
	sf := new(StiffForce)
	sf.k = k
	return sf
}
func (mod *StiffForce) Act(in, out *poly.Chain) {
	for i, _ := range in.Loc[2:len(in.Loc)-2] {
		//note that i+2 is the index for the in.Loc array slice
		out.Vel[i+2] = out.Vel[i+2].Add(in.Loc[i+2].Mul(2).Sub(in.Loc[i+4]).Sub(in.Loc[i]).Mul(mod.k))
	}
	out.Vel[1] = out.Vel[1].Add(in.Loc[1].Sub(in.Loc[3]).Mul(mod.k))
	out.Vel[0] = out.Vel[0].Add(in.Loc[0].Sub(in.Loc[2]).Mul(mod.k))
	out.Vel[len(out.Vel)-2] = out.Vel[len(out.Vel)-2].Add(in.Loc[len(out.Loc)-2].Sub(in.Loc[len(out.Loc)-4]).Mul(mod.k))
	out.Vel[len(out.Vel)-1] = out.Vel[len(out.Vel)-1].Add(in.Loc[len(out.Loc)-1].Sub(in.Loc[len(out.Loc)-3]).Mul(mod.k))
}
type PinForce struct {
	k float64
	Target int
	Loc *vector.D3
}
func NewPinForce(k float64, target int, loc *vector.D3) Modifier {
	pf := new(PinForce)
	pf.k = k
	pf.Target = target
	pf.Loc = loc
	return pf
}
func (mod *PinForce) Act(in, out *poly.Chain) {
	out.Vel[mod.Target] = out.Vel[mod.Target].Add(mod.Loc.Sub(in.Loc[mod.Target]).Mul(mod.k))
}
type KinesinForce struct {
	k float64
}
func NewKinesinForce(k float64) Modifier {
	kf := new(KinesinForce)
	kf.k = k
	return kf
}
func (mod *KinesinForce) Act(in, out *poly.Chain) {
	for i, _ := range in.Loc[1:len(in.Loc)-1] {
		out.Vel[i+1] = out.Vel[i+1].Add(in.Loc[i].Sub(in.Loc[i+2]).Mul(mod.k))
	}
	out.Vel[0] = out.Vel[0].Add(in.Loc[0].Sub(in.Loc[1]).Mul(mod.k))
	out.Vel[len(out.Vel)-1] = out.Vel[len(out.Vel)-1].Add(in.Loc[len(in.Loc)-2].Sub(in.Loc[len(in.Loc)-1]).Mul(mod.k))
}
type ConstForce struct {
	f *vector.D3
}
func NewConstForce(f *vector.D3) Modifier {
	cf := new(ConstForce)
	cf.f = f
	return cf
}
func (mod *ConstForce) Act(in, out *poly.Chain) {
	for i,_ := range in.Loc {
		out.Vel[i] = out.Vel[i].Add(mod.f)
	}
}