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
	Act(input, output []*poly.Chain)
}


type RK4 struct {
	h,h2,h6 float64
	sys [][]*poly.Chain
	Sys, sys2,sys3,sys4 *poly.Chain
	Mods []Modifier
}

func (rk *RK4) Init(chains []*poly.Chain) {
	rk.sys = make([][]*poly.Chain, 4)
	rk.sys[0] = make([]*poly.Chain,len(chains))
	rk.sys[1] = make([]*poly.Chain,len(chains))
	rk.sys[2] = make([]*poly.Chain,len(chains))
	rk.sys[3] = make([]*poly.Chain,len(chains))
	for i , chain := range chains {
		rk.sys[0][i] = chain
		rk.sys[1][i] = poly.New(chain.GetLength())
		rk.sys[2][i] = poly.New(chain.GetLength())
		rk.sys[3][i] = poly.New(chain.GetLength())	
	}
	rk.Mods = make([]Modifier, 1)
	rk.Mods[0] = NewSpringForce(100)
	rk.Mods = append(rk.Mods, NewStiffForce(10))
	rk.Mods = append(rk.Mods, NewOseenTensor(.2,rk.sys[0]))
	rk.Mods = append(rk.Mods, NewKinesinForce(1))
	rk.Mods = append(rk.Mods, NewPinForce(100, 0, 0, vector.NewD3(0,0,0)))
	rk.Mods = append(rk.Mods, NewPinForce(100, 1, 0, vector.NewD3(1,1,0)))
	//rk.Mods = append(rk.Mods, NewConstForce(vector.NewD3(0,0,1)))
	rk.h = .001
	rk.h2 = rk.h/2
	rk.h6 = rk.h/6
}

func (rk *RK4) Step() []*poly.Chain {
	//take a step using RK4
	for n := 0; n < 100 ; n ++ {
	//clear the forces
	for j, _ := range rk.sys {
		for n, _ := range rk.sys[j] {
			for i, _ := range rk.sys[j][n].Vel {
				rk.sys[j][n].Vel[i].Zero()
			}
		}
	}
	//calculate forces at the initial point (k1)
	for _, mod := range rk.Mods {
		mod.Act(rk.sys[0], rk.sys[0])
	}
	//determine the locations for the first midpoint step
	for n, _ := range rk.sys[0] {
		for i, _ := range rk.sys[1][n].Loc {
			rk.sys[1][n].Loc[i] = rk.sys[0][n].Loc[i].Add(rk.sys[0][n].Vel[i].Mul(rk.h2))
		}
	}
	//calculate forces for the first midpoint (k2)
	for _, mod := range rk.Mods {
		mod.Act(rk.sys[1], rk.sys[1])
	}
	//determine the locations foe the second midpoint step
	for n, _ := range rk.sys[0] {
		for i, _ := range rk.sys[2][n].Loc {
			rk.sys[2][n].Loc[i] = rk.sys[0][n].Loc[i].Add(rk.sys[1][n].Vel[i].Mul(rk.h2))
		}
	}
	//calculate forces for the second midpoint (k3)
	for _, mod := range rk.Mods {
		mod.Act(rk.sys[2], rk.sys[2])
	}
	//determine the locations foe the second midpoint step
	for n, _ := range rk.sys[0] {
		for i, _ := range rk.sys[3][n].Loc {
			rk.sys[3][n].Loc[i] = rk.sys[0][n].Loc[i].Add(rk.sys[2][n].Vel[i].Mul(rk.h))
		}
	}
	//calculate forces for the endpoint
	for _, mod := range rk.Mods {
		mod.Act(rk.sys[3], rk.sys[3])
	}
	//apply the transform to the primary system
	for n, _ := range rk.sys[0] {
		for i, _ := range rk.sys[0][n].Loc {
			rk.sys[0][n].Loc[i] = rk.sys[0][n].Loc[i].Add(rk.sys[0][n].Vel[i].Add( rk.sys[1][n].Vel[i].Add(rk.sys[2][n].Vel[i]).Mul(2) ).Add( rk.sys[3][n].Vel[i] ).Mul(rk.h6) )
		}
	}
	}//end multiplicity loop
	return rk.sys[0]
}


type SpringForce struct {
	k float64
}
func NewSpringForce(k float64) Modifier {
	sf := new(SpringForce)
	sf.k = k
	return sf
}
func (mod *SpringForce) Act(in, out []*poly.Chain) {
	var f1, f2 *vector.D3
	for n, _ := range in{
		for i, r := range in[n].Loc[1:len(in[n].Loc)-1] {
			f1 = in[n].Loc[i].Sub(r)
			f2 = r.Sub(in[n].Loc[i+2])
			//out.Vel[i] = vector.NewD3(2,2,2)
			out[n].Vel[i+1] = out[n].Vel[i+1].Add(f1.Sub(f1.Normalize()).Sub(f2.Sub(f2.Normalize())).Mul(mod.k))
		}
		f1 = in[n].Loc[1].Sub(in[n].Loc[0])
		f2 = in[n].Loc[len(in[n].Loc)-2].Sub(in[n].Loc[len(in[n].Loc)-1])
		out[n].Vel[0] = out[n].Vel[0].Add(f1.Sub(f1.Normalize()).Mul(mod.k))
		out[n].Vel[len(in[n].Vel)-1] = out[n].Vel[len(in[n].Vel)-1].Add(f2.Sub(f2.Normalize()).Mul(mod.k))
	}
}
type StiffForce struct {
	k float64
}
func NewStiffForce(k float64) Modifier{
	sf := new(StiffForce)
	sf.k = k
	return sf
}
func (mod *StiffForce) Act(in, out []*poly.Chain) {
	for n, _ := range in{
		for i, _ := range in[n].Loc[2:len(in[n].Loc)-2] {
			//note that i+2 is the index for the in.Loc array slice
			out[n].Vel[i+2] = out[n].Vel[i+2].Add(in[n].Loc[i+2].Mul(2).Sub(in[n].Loc[i+4]).Sub(in[n].Loc[i]).Mul(mod.k))
		}
		out[n].Vel[1] = out[n].Vel[1].Add(in[n].Loc[1].Sub(in[n].Loc[3]).Mul(mod.k))
		out[n].Vel[0] = out[n].Vel[0].Add(in[n].Loc[0].Sub(in[n].Loc[2]).Mul(mod.k))
		out[n].Vel[len(out[n].Vel)-2] = out[n].Vel[len(out[n].Vel)-2].Add(in[n].Loc[len(out[n].Loc)-2].Sub(in[n].Loc[len(out[n].Loc)-4]).Mul(mod.k))
		out[n].Vel[len(out[n].Vel)-1] = out[n].Vel[len(out[n].Vel)-1].Add(in[n].Loc[len(out[n].Loc)-1].Sub(in[n].Loc[len(out[n].Loc)-3]).Mul(mod.k))
	}
}
type PinForce struct {
	k float64
	chainIndex, linkIndex int
	Loc *vector.D3
}
func NewPinForce(k float64, chainIndex int, linkIndex int, loc *vector.D3) Modifier {
	pf := new(PinForce)
	pf.k = k
	pf.chainIndex = chainIndex
	pf.linkIndex = linkIndex
	pf.Loc = loc
	return pf
}
func (mod *PinForce) Act(in, out []*poly.Chain) {
	out[mod.chainIndex].Vel[mod.linkIndex] = out[mod.chainIndex].Vel[mod.linkIndex].Add(mod.Loc.Sub(in[mod.chainIndex].Loc[mod.linkIndex]).Mul(mod.k))
}
type KinesinForce struct {
	k float64
}
func NewKinesinForce(k float64) Modifier {
	kf := new(KinesinForce)
	kf.k = k
	return kf
}
func (mod *KinesinForce) Act(in, out []*poly.Chain) {
	for n, _ := range in {
		for i, _ := range in[n].Loc[1:len(in[n].Loc)-1] {
			out[n].Vel[i+1] = out[n].Vel[i+1].Add(in[n].Loc[i].Sub(in[n].Loc[i+2]).Mul(mod.k))
		}
		out[n].Vel[0] = out[n].Vel[0].Add(in[n].Loc[0].Sub(in[n].Loc[1]).Mul(mod.k))
		out[n].Vel[len(out[n].Vel)-1] = out[n].Vel[len(out[n].Vel)-1].Add(in[n].Loc[len(in[n].Loc)-2].Sub(in[n].Loc[len(in[n].Loc)-1]).Mul(mod.k))
	}
}
type ConstForce struct {
	f *vector.D3
}
func NewConstForce(f *vector.D3) Modifier {
	cf := new(ConstForce)
	cf.f = f
	return cf
}
func (mod *ConstForce) Act(in, out []*poly.Chain) {
	for n,_ := range in {
		for i,_ := range in[n].Loc {
			out[n].Vel[i] = out[n].Vel[i].Add(mod.f)
		}
	}
}
type OseenTensor struct {
	k, norm, clamp float64
	vec, dif *vector.D3
	temp []*poly.Chain
}
func NewOseenTensor(k float64, system []*poly.Chain) Modifier {
	ot := new(OseenTensor)
	ot.k = k
	ot.clamp = 10
	ot.vec = vector.NewD3(0,0,0)
	ot.temp = make([]*poly.Chain, len(system))
	for n,chain := range system {
		ot.temp[n] = poly.New(chain.GetLength())
	}
	return ot
}
func (mod *OseenTensor) Act(in, out []*poly.Chain) {
	//madness
	//for each monomer
	for n,_ := range in {
		for i, _ := range in[n].Loc {
			mod.vec.Zero()
			for m,_ := range in {
				for j,_ := range in[m].Loc {
					if m == n && i == j {
						mod.vec = mod.vec.Add(in[n].Vel[i])
						continue
					}
					mod.dif = in[m].Loc[j].Sub(in[n].Loc[i])
					mod.norm = mod.dif.Norm()
					if mod.norm == 0 {
						continue
					}
					mod.vec = mod.vec.Add( in[m].Vel[j].Add( mod.dif.Mul(mod.dif.Dot(in[m].Vel[j]) ).Mul(1.0/(mod.norm*mod.norm)) ).Mul(mod.k/mod.norm) )
				}
			}
			//the force from the fluid saturates to avoid problems with finite timestep
			mod.norm = mod.vec.Norm()
			if mod.norm > mod.clamp {
				mod.vec = mod.vec.Mul(mod.clamp/mod.norm)
			}
			mod.temp[n].Vel[i].Copy(mod.vec)
		}
	}
	for n,_ := range mod.temp {
		for i,_ := range mod.temp[n].Vel {
			out[n].Vel[i].Copy(mod.temp[n].Vel[i])
		}
	}
}