package physics.calibration;

import java.util.*;

/** Fits an effective tread/surface sliding-onset coefficient; validation trials never fit it. */
public final class GripCalibrator {
    private GripCalibrator() {}
    public record Reading(String trial,String set,double normalN,double forceN) {
        public Reading {
            if(trial==null||trial.isBlank()||trial.length()>100||trial.contains("\n")||trial.contains("\r"))throw new IllegalArgumentException("Give each physical trial a short nonblank ID.");
            if(!"fit".equals(set)&&!"validate".equals(set))throw new IllegalArgumentException("Choose fit or validate for each whole trial.");
            if(!Double.isFinite(normalN)||normalN<.000001||normalN>100000||!Double.isFinite(forceN)||forceN<0||forceN>100000)throw new IllegalArgumentException("Normal load must be 0.000001–100000 N and force 0–100000 N, both finite.");
        }
    }
    public record Criteria(int minFitTrials,int minValidationTrials,double minLoadVariation,
                           double maxRelativeRmse,double maxTrialRelativeRmse) {
        public Criteria {
            if(minFitTrials<3||minFitTrials>100||minValidationTrials<2||minValidationTrials>100
                ||!Double.isFinite(minLoadVariation)||minLoadVariation<.05||minLoadVariation>1
                ||!Double.isFinite(maxRelativeRmse)||maxRelativeRmse<.01||maxRelativeRmse>.5
                ||!Double.isFinite(maxTrialRelativeRmse)||maxTrialRelativeRmse<.01||maxTrialRelativeRmse>.75)
                throw new IllegalArgumentException("Grip criteria: at least 3 fitting/2 validation trials; load variation 0.05–1; relative RMSE 0.01–0.5; per-trial error 0.01–0.75.");
        }
        public static Criteria defaults(){return new Criteria(3,2,.2,.15,.25);}
    }
    public record Result(double effectiveMu,double wheelCoefficient,int fitTrials,int validationTrials,
                         double loadVariation,double fitRmseN,double validationRmseN,
                         double fitRelativeRmse,double validationRelativeRmse,double worstTrialRelativeRmse,
                         boolean accepted,List<String> issues) {
        public Result {issues=List.copyOf(issues);}
    }
    private record Error(double rmse,double relative,double worst,String worstTrial) {}
    public static Result fit(List<Reading> readings,double surfaceCoefficient,Criteria criteria) {
        if(!Double.isFinite(surfaceCoefficient)||surfaceCoefficient<.000001||surfaceCoefficient>2)throw new IllegalArgumentException("Reference surface friction must be 0.000001–2.");
        if(readings==null||readings.isEmpty()||readings.size()>1000)throw new IllegalArgumentException("Provide 1–1000 physical readings.");
        var trials=new LinkedHashMap<String,List<Reading>>();
        for(var r:readings){Objects.requireNonNull(r,"Missing reading");var trial=trials.computeIfAbsent(r.trial(),id->new ArrayList<>());if(!trial.isEmpty()&&!trial.get(0).set().equals(r.set()))throw new IllegalArgumentException("Trial "+r.trial()+" appears in both sets. Reserve entire new trials for validation.");trial.add(r);}
        var fit=trials.values().stream().filter(t->t.get(0).set().equals("fit")).toList();
        var validation=trials.values().stream().filter(t->t.get(0).set().equals("validate")).toList();
        if(fit.size()<criteria.minFitTrials()||validation.size()<criteria.minValidationTrials())throw new IllegalArgumentException("Need "+criteria.minFitTrials()+" fitting and "+criteria.minValidationTrials()+" independent validation trials; found "+fit.size()+" and "+validation.size()+". Repeated rows in one trial count once.");
        double numerator=0,denominator=0,min=Double.POSITIVE_INFINITY,max=0;
        for(var trial:fit){double n=trial.stream().mapToDouble(Reading::normalN).average().orElseThrow(),f=trial.stream().mapToDouble(Reading::forceN).average().orElseThrow();numerator+=n*f;denominator+=n*n;min=Math.min(min,n);max=Math.max(max,n);}
        double mu=numerator/denominator,wheel=mu/surfaceCoefficient,variation=(max-min)/max;
        var fitError=error(fit,mu);var validationError=error(validation,mu);double worst=Math.max(fitError.worst(),validationError.worst());var issues=new ArrayList<String>();
        if(variation<criteria.minLoadVariation())issues.add("Fitting normal loads vary too little. Use additional known load levels; do not change validation trials into fitting trials.");
        if(mu<=0)issues.add("No positive measured grip. Check force readings and sliding onset; use manual settings for an intentional zero-grip assumption.");
        if(wheel>2)issues.add("Fitted wheel coefficient exceeds 2. Check load/force units and the reference surface coefficient; the result is not clamped.");
        if(fitError.relative()>criteria.maxRelativeRmse())issues.add("Fitting residual exceeds the saved tolerance. Check repeatability, contact direction and load measurement.");
        if(validationError.relative()>criteria.maxRelativeRmse())issues.add("Independent validation residual exceeds the saved tolerance. Collect fresh trials and investigate changed surface or test conditions.");
        if(worst>criteria.maxTrialRelativeRmse())issues.add("Trial '"+(fitError.worst()>=validationError.worst()?fitError.worstTrial():validationError.worstTrial())+"' exceeds the per-trial error tolerance. Inspect its readings and conditions.");
        return new Result(mu,wheel,fit.size(),validation.size(),variation,fitError.rmse(),validationError.rmse(),fitError.relative(),validationError.relative(),worst,issues.isEmpty(),issues);
    }
    private static Error error(List<List<Reading>> trials,double mu) {
        double squared=0,forceSquared=0,worst=0;String worstTrial="";
        for(var trial:trials){double residual=0,force=0;for(var r:trial){residual+=Math.pow(r.forceN()-mu*r.normalN(),2);force+=r.forceN()*r.forceN();}residual/=trial.size();force/=trial.size();squared+=residual;forceSquared+=force;double relative=Math.sqrt(residual)/Math.max(1e-9,Math.sqrt(force));if(relative>worst){worst=relative;worstTrial=trial.get(0).trial();}}
        return new Error(Math.sqrt(squared/trials.size()),Math.sqrt(squared)/Math.max(1e-9,Math.sqrt(forceSquared)),worst,worstTrial);
    }
}
