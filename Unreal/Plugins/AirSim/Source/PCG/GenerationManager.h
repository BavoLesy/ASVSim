// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Math/Transform.h"
#include "GenerationManager.generated.h"

UCLASS()
class AIRSIM_API AGenerationManager : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AGenerationManager();

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Setup")
	TArray<FTransform> road;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Setup")
	TArray<FTransform> aleft;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Setup")
	TArray<FTransform> aright;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Setup")
	TArray<FTransform> temp;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Setup")
	int aseed;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Setup")
	FRandomStream stream;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Setup")
	TArray<AActor*> generated;

	UFUNCTION(BlueprintCallable, Category = "LevelSetup")
	bool HavenStep(UPARAM(ref) TArray<FTransform>& haven, float mina, float maxa, float mind, float maxd);

	UFUNCTION(BlueprintCallable, Category = "LevelSetup")
	void WallStep(UPARAM(ref) TArray<FTransform>& haven, UPARAM(ref) TArray<FTransform>& wall, float min, float max);

	UFUNCTION(BlueprintCallable, Category = "LevelSetup")
	void GenerateWalls(UPARAM(ref) TArray<FTransform>& points, bool inlets, float min, float max);

	UFUNCTION(BlueprintCallable, Category = "LevelSetup")
	void InletFunction(FTransform begin, FTransform end, FRotator rotation, float min, float max, float length);

	UFUNCTION(BlueprintCallable, Category = "LevelSetup")
	void InletGeneration(UPARAM(ref) TArray<FTransform>& points);

	UFUNCTION(BlueprintCallable, Category = "LevelSetup")
	void Clear();

	UFUNCTION(BlueprintCallable, Category = "LevelSetup")
	void SetSeed(int32 seed);

	UFUNCTION(BlueprintCallable, Category = "LevelSetup")
	void generateTerrain(FString type, float mina, float maxa, float mind, float maxd, int length);

	UFUNCTION(BlueprintCallable, Category = "LevelSetup")
	void getGoal(int32 distance,FVector& location, FVector& left, FVector& right);

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

};
