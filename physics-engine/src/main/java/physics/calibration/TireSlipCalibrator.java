package physics.calibration;

import physics.TireFriction;
import java.util.*;

/** Bounded steady longitudinal brush fit. Entire physical trials stay in one partition. */
public final class TireSlipCalibrator {
    private TireSlipCalibrator() {}
    public record Reading(String trial,String set,double wheelMps,double hubMps,double normalN,double forceN) {
        public Reading {
            if(trial==null||trial.isBlank()||trial.length()>100||trial.contains("\n")||trial.contains("\r"))throw new IllegalArgumentException("Give every steady physical trial a short ID");
            if(!List.of("fit","validate").contains(set==null?"":set))throw new IllegalArgumentException("Choose fit or validate for each entire trial");
            for(double x:new double[]{wheelMps,hubMps,normalN,forceN})if(!Double.isFinite(x))throw new IllegalArgumentException("Tire readings must be finite SI values");
            if(Math.abs(wheelMps)>20||Math.abs(hubMps)>20||normalN<.000001||normalN>100000||Math.abs(forceN)>100000)throw new IllegalArgumentException("Speeds must be within ±20 m/s, normal load positive up to 100000 N, force within ±100000 N");
            if(forceN*(wheelMps-hubMps)<0)throw new IllegalArgumentException("Longitudinal tire force must have the sign of wheel surface speed minus independent hub speed");
        }
        public double slip(){return wheelMps-hubMps;}
    }
    public record Bounds(double minStatic,double maxStatic,double minSliding,double maxSliding,double minStiffness,double maxStiffness,double minTransition,double maxTransition) {
        public Bounds {
            double[] lo={minStatic,minSliding,minStiffness,minTransition},hi={maxStatic,maxSliding,maxStiffness,maxTransition},limit={2,2,100000,5};
            for(int i=0;i<4;i++)if(!Double.isFinite(lo[i])||!Double.isFinite(hi[i])||lo[i]<=0||hi[i]>limit[i]||hi[i]<=lo[i])throw new IllegalArgumentException("Fit bounds need positive ordered limits: friction ≤2, stiffness ≤100000, transition ≤5");
            if(minSliding>=maxStatic)throw new IllegalArgumentException("Sliding bounds must allow sliding friction below static friction");
        }
        public static Bounds defaults(){return new Bounds(.1,2,.05,2,1,2000,.01,1);}
        double[] lower(){return new double[]{minStatic,minSliding,minStiffness,minTransition};}
        double[] upper(){return new double[]{maxStatic,maxSliding,maxStiffness,maxTransition};}
    }
    public record Criteria(int minFitTrials,int minValidationTrials,double minLoadVariation,double maxRelativeRmse,double maxTrialError,double maxSlipSpread,double maxLoadSpread,double minSensitivity,double minInformationEigenvalue) {
        public Criteria {
            if(minFitTrials<12||minFitTrials>500||minValidationTrials<6||minValidationTrials>500)throw new IllegalArgumentException("Need at least 12 fitting and 6 reserved validation trials");
            double[] xs={minLoadVariation,maxRelativeRmse,maxTrialError,maxSlipSpread,maxLoadSpread,minSensitivity,minInformationEigenvalue};
            double[] lows={.05,.01,.01,.01,.01,.001,.0001},highs={1,.5,.75,.25,.25,.1,.1};
            for(int i=0;i<xs.length;i++)if(!Double.isFinite(xs[i])||xs[i]<lows[i]||xs[i]>highs[i])throw new IllegalArgumentException("Invalid tire fit quality criterion");
        }
        public static Criteria defaults(){return new Criteria(12,6,.2,.15,.25,.1,.05,.01,.001);}
    }
    public record Result(TireFriction.Spec spec,int fitTrials,int validationTrials,double loadVariation,double fitRmseN,double validationRmseN,double fitRelativeRmse,double validationRelativeRmse,double worstTrialError,double minSensitivity,double informationEigenvalue,boolean accepted,List<String> issues) {public Result{issues=List.copyOf(issues);}}
    private record Trial(String id,String set,List<Reading> rows,Reading mean) {}
    private record Error(double rmse,double relative,double worst,String trial) {}
    private record Candidate(double[] x,double loss) {}
    public static Result fit(List<Reading> readings,double lateralScale,Bounds bounds,Criteria criteria) {
        if(readings==null||readings.isEmpty()||readings.size()>1000)throw new IllegalArgumentException("Provide 1–1000 steady tire readings");
        if(!Double.isFinite(lateralScale)||lateralScale<0||lateralScale>1)throw new IllegalArgumentException("Lateral scale remains an explicit 0–1 assumption");
        var grouped=new LinkedHashMap<String,List<Reading>>();
        for(var r:readings){var list=grouped.computeIfAbsent(r.trial(),k->new ArrayList<>());if(!list.isEmpty()&&!list.get(0).set().equals(r.set()))throw new IllegalArgumentException("Trial "+r.trial()+" appears in both sets; reserve entire new trials");list.add(r);}
        var fit=new ArrayList<Trial>();var validation=new ArrayList<Trial>();var issues=new ArrayList<String>();
        for(var entry:grouped.entrySet()) {
            var rows=entry.getValue();var first=rows.get(0);var mean=new Reading(first.trial(),first.set(),average(rows,0),average(rows,1),average(rows,2),average(rows,3));
            double slipSpread=Math.sqrt(rows.stream().mapToDouble(r->Math.pow(r.slip()-mean.slip(),2)).average().orElseThrow())/Math.max(.001,Math.abs(mean.slip()));
            double loadSpread=Math.sqrt(rows.stream().mapToDouble(r->Math.pow(r.normalN()-mean.normalN(),2)).average().orElseThrow())/mean.normalN();
            if(slipSpread>criteria.maxSlipSpread()||loadSpread>criteria.maxLoadSpread())issues.add("Trial "+entry.getKey()+" is not steady within saved slip/load tolerances; separate stable test conditions");
            var t=new Trial(entry.getKey(),first.set(),List.copyOf(rows),mean);(first.set().equals("fit")?fit:validation).add(t);
        }
        if(fit.size()<criteria.minFitTrials()||validation.size()<criteria.minValidationTrials())throw new IllegalArgumentException("Need "+criteria.minFitTrials()+" fitting and "+criteria.minValidationTrials()+" independent validation trials; repeated rows count once");
        for(var set:List.of(fit,validation))if(set.stream().filter(t->t.mean.slip()>0).count()<3||set.stream().filter(t->t.mean.slip()<0).count()<3)issues.add("Both fitting and validation need at least three positive-slip and three negative-slip trials");
        double min=fit.stream().mapToDouble(t->t.mean.normalN()).min().orElseThrow(),max=fit.stream().mapToDouble(t->t.mean.normalN()).max().orElseThrow(),variation=(max-min)/max;
        if(variation<criteria.minLoadVariation())issues.add("Use a wider range of known normal loads in fitting trials");
        var spec=optimize(fit,lateralScale,bounds);var a=error(fit,spec);var b=error(validation,spec);
        if(a.relative>criteria.maxRelativeRmse())issues.add("Fitting force error exceeds tolerance");
        if(b.relative>criteria.maxRelativeRmse())issues.add("Independent validation force error exceeds tolerance");
        var worst=a.worst>=b.worst?a:b;if(worst.worst>criteria.maxTrialError())issues.add("Trial "+worst.trial+" exceeds the individual error tolerance");
        double[] values={spec.staticMu(),spec.slidingMu(),spec.stiffnessNPerMps(),spec.transitionMps()},lo=bounds.lower(),hi=bounds.upper();
        for(int i=0;i<4;i++)if(Math.log(values[i]/lo[i])<.005*Math.log(hi[i]/lo[i])||Math.log(hi[i]/values[i])<.005*Math.log(hi[i]/lo[i]))issues.add("Fit reaches a search boundary for "+List.of("static_mu","sliding_mu","stiffness_n_per_mps","transition_mps").get(i)+"; check readings and bounds");
        var identification=information(fit,spec);
        if(identification[0]<criteria.minSensitivity())issues.add("A fitted parameter has too little force sensitivity; include low slip, transition and high-slip plateau trials");
        if(identification[1]<criteria.minInformationEigenvalue())issues.add("Fitted parameters are correlated or unidentifiable; add independent slip/load conditions");
        return new Result(spec,fit.size(),validation.size(),variation,a.rmse,b.rmse,a.relative,b.relative,worst.worst,identification[0],identification[1],issues.isEmpty(),issues);
    }
    private static double average(List<Reading> rows,int column){return rows.stream().mapToDouble(r->switch(column){case 0->r.wheelMps;case 1->r.hubMps;case 2->r.normalN;default->r.forceN;}).average().orElseThrow();}
    private static TireFriction.Spec spec(double[] x,double lateral){return new TireFriction.Spec(Math.exp(x[0]),Math.exp(x[1]),lateral,Math.exp(x[2]),Math.exp(x[3]));}
    private static double loss(List<Trial> trials,TireFriction.Spec s){return trials.stream().mapToDouble(t->Math.pow((TireFriction.steadyLongitudinal(s,t.mean.slip(),t.mean.normalN())-t.mean.forceN())/t.mean.normalN(),2)).average().orElseThrow();}
    private static TireFriction.Spec optimize(List<Trial> trials,double lateral,Bounds bounds) {
        double[] lo=Arrays.stream(bounds.lower()).map(Math::log).toArray(),hi=Arrays.stream(bounds.upper()).map(Math::log).toArray();var best=new ArrayList<Candidate>();
        for(int a=0;a<7;a++)for(int b=0;b<7;b++)for(int k=0;k<8;k++)for(int t=0;t<8;t++) {
            double[] x={lo[0]+(hi[0]-lo[0])*a/6,lo[1]+(hi[1]-lo[1])*b/6,lo[2]+(hi[2]-lo[2])*k/7,lo[3]+(hi[3]-lo[3])*t/7};if(x[1]>x[0])continue;
            double score=loss(trials,spec(x,lateral));best.add(new Candidate(x,score));best.sort(Comparator.comparingDouble(Candidate::loss));if(best.size()>8)best.remove(8);
        }
        Candidate answer=best.get(0);
        for(var seed:best) {
            double[] x=seed.x.clone(),step=new double[4];for(int i=0;i<4;i++)step[i]=(hi[i]-lo[i])*.12;double score=seed.loss;
            for(int iteration=0;iteration<240;iteration++) {
                boolean improved=false;for(int i=0;i<4;i++)for(int sign:new int[]{-1,1}) {
                    double[] next=x.clone();next[i]=Math.max(lo[i],Math.min(hi[i],x[i]+sign*step[i]));if(next[1]>next[0])continue;
                    double n=loss(trials,spec(next,lateral));if(n<score){x=next;score=n;improved=true;}
                }
                if(!improved){for(int i=0;i<4;i++)step[i]*=.5;if(Arrays.stream(step).max().orElseThrow()<1e-8)break;}
            }
            if(score<answer.loss)answer=new Candidate(x,score);
        }
        return spec(answer.x,lateral);
    }
    private static Error error(List<Trial> trials,TireFriction.Spec spec) {
        double squared=0,forceSquared=0,worst=0;String name="";
        for(var t:trials){double residual=t.rows.stream().mapToDouble(r->Math.pow(TireFriction.steadyLongitudinal(spec,r.slip(),r.normalN())-r.forceN(),2)).average().orElseThrow();double force=t.rows.stream().mapToDouble(r->r.forceN()*r.forceN()).average().orElseThrow();squared+=residual;forceSquared+=force;double rel=Math.sqrt(residual)/Math.max(1e-6,Math.sqrt(force));if(rel>worst){worst=rel;name=t.id;}}
        return new Error(Math.sqrt(squared/trials.size()),Math.sqrt(squared)/Math.max(1e-6,Math.sqrt(forceSquared)),worst,name);
    }
    /** Log-parameter force sensitivities and minimum eigenvalue of column-normalized information. */
    private static double[] information(List<Trial> trials,TireFriction.Spec s) {
        double[] x={Math.log(s.staticMu()),Math.log(s.slidingMu()),Math.log(s.stiffnessNPerMps()),Math.log(s.transitionMps())};double[][] j=new double[trials.size()][4];double[] norm=new double[4];
        for(int column=0;column<4;column++) {
            double[] up=x.clone(),down=x.clone();up[column]+=.0001;down[column]-=.0001;
            // Compute the envelope directly so the derivative can cross static == sliding.
            for(int r=0;r<trials.size();r++){var m=trials.get(r).mean;j[r][column]=(prediction(up,m)-prediction(down,m))/.0002;norm[column]+=j[r][column]*j[r][column];}
        }
        double minimum=Arrays.stream(norm).map(v->Math.sqrt(v/trials.size())).min().orElseThrow();double[][] g=new double[4][4];
        for(int a=0;a<4;a++)for(int b=0;b<4;b++)for(var row:j)g[a][b]+=row[a]*row[b]/Math.max(1e-30,Math.sqrt(norm[a]*norm[b]));
        for(int iteration=0;iteration<80;iteration++){int a=0,b=1;for(int i=0;i<4;i++)for(int k=i+1;k<4;k++)if(Math.abs(g[i][k])>Math.abs(g[a][b])){a=i;b=k;}if(Math.abs(g[a][b])<1e-12)break;double angle=.5*Math.atan2(2*g[a][b],g[b][b]-g[a][a]),c=Math.cos(angle),sn=Math.sin(angle),aa=g[a][a],bb=g[b][b],ab=g[a][b];for(int i=0;i<4;i++)if(i!=a&&i!=b){double ia=g[i][a],ib=g[i][b];g[i][a]=g[a][i]=c*ia-sn*ib;g[i][b]=g[b][i]=sn*ia+c*ib;}g[a][a]=c*c*aa-2*c*sn*ab+sn*sn*bb;g[b][b]=sn*sn*aa+2*c*sn*ab+c*c*bb;g[a][b]=g[b][a]=0;}
        double eigen=Double.POSITIVE_INFINITY;for(int i=0;i<4;i++)eigen=Math.min(eigen,g[i][i]);return new double[]{minimum,Math.max(0,eigen)};
    }
    private static double prediction(double[] x,Reading r){double mu=Math.exp(x[1])+(Math.exp(x[0])-Math.exp(x[1]))*Math.exp(-Math.pow(r.slip()/Math.exp(x[3]),2));return Math.copySign(Math.min(Math.exp(x[2])*Math.abs(r.slip()),mu*r.normalN()),r.slip())/r.normalN();}
}
